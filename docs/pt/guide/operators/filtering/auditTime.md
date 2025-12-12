---
description: auditTime é um operador de filtragem RxJS que espera um tempo especificado após um valor ser emitido e emite o último valor dentro desse período. É ideal quando você quer amostrar periodicamente o último estado de eventos de alta frequência como rastreamento de posição de rolagem, redimensionamento de janela e movimento do mouse. É importante entender a diferença de throttleTime e debounceTime e usá-los apropriadamente.
titleTemplate: ':title | RxJS'
---

# auditTime - Emitir Último Valor Após Tempo Especificado

O operador `auditTime` espera por um **tempo especificado** após um valor ser emitido e emite o **último valor** dentro desse período de tempo. Em seguida, espera pelo próximo valor.


## 🔰 Sintaxe Básica e Uso

```ts
import { fromEvent } from 'rxjs';
import { auditTime } from 'rxjs';

fromEvent(document, 'click').pipe(
  auditTime(1000)
).subscribe(() => console.log('Clique!'));
```

**Fluxo de operação**:
1. Primeiro clique ocorre
2. Espera 1 segundo (cliques durante este tempo são registrados mas não emitidos)
3. Emite o último clique após 1 segundo
4. Espera pelo próximo clique

[🌐 Documentação Oficial RxJS - `auditTime`](https://rxjs.dev/api/operators/auditTime)


## 🆚 Contraste com throttleTime

`throttleTime` e `auditTime` são similares, mas emitem valores diferentes.

```ts
import { interval } from 'rxjs';
import { throttleTime, auditTime, take } from 'rxjs';

const source$ = interval(300).pipe(take(10)); // 0, 1, 2, 3, 4, 5, 6, 7, 8, 9

// throttleTime: Emitir o primeiro valor
source$.pipe(
  throttleTime(1000)
).subscribe(console.log);
// Saída: 0, 4, 8 (primeiro valor de cada período)

// auditTime: Emitir o último valor
source$.pipe(
  auditTime(1000)
).subscribe(console.log);
// Saída: 3, 6, 9 (último valor de cada período)
```

**Comparação de linha do tempo**:
```
Fonte:      0--1--2--3--4--5--6--7--8--9--|
            |        |        |
throttle:   0--------4--------8------------|
            (primeiro)  (primeiro)  (primeiro)

audit:      -------3--------6--------9----|
                  (último)   (último)   (último)
```

| Operador | Valor Emitido | Tempo de Emissão | Caso de Uso |
|---|---|---|---|
| `throttleTime(ms)` | **Primeiro** valor dentro do período | Ao receber valor | Reação imediata necessária |
| `auditTime(ms)` | **Último** valor dentro do período | No fim do período | Último estado necessário |
| `debounceTime(ms)` | **Último** valor após silêncio | Após a entrada parar | Esperar conclusão de entrada |


## 💡 Padrões de Uso Típicos

1. **Otimização de Redimensionamento de Janela**
   ```ts
   import { fromEvent } from 'rxjs';
   import { auditTime } from 'rxjs';

   fromEvent(window, 'resize').pipe(
     auditTime(200) // Obter último tamanho a cada 200ms
   ).subscribe(() => {
     console.log(`Tamanho da janela: ${window.innerWidth}x${window.innerHeight}`);
   });
   ```

2. **Rastreamento de Posição de Rolagem**
   ```ts
   import { fromEvent } from 'rxjs';
   import { auditTime, map } from 'rxjs';

   fromEvent(window, 'scroll').pipe(
     auditTime(100),
     map(() => ({
       scrollY: window.scrollY,
       scrollX: window.scrollX
     }))
   ).subscribe(position => {
     console.log(`Posição de rolagem: Y=${position.scrollY}, X=${position.scrollX}`);
   });
   ```


## 🎯 Diferença de debounceTime

`auditTime` e `debounceTime` ambos **emitem o último valor**, mas o **tempo é completamente diferente**.

### Diferença Principal

| Operador | Comportamento | Caso de Uso |
|---|---|---|
| `auditTime(ms)` | **Sempre emite após ms** uma vez que o valor chega (mesmo se a entrada continuar) | Quer amostrar periodicamente |
| `debounceTime(ms)` | Emite após ms **depois que a entrada para** | Quer esperar conclusão de entrada |

### Exemplo Concreto: Diferença em Entrada de Pesquisa

```ts
import { fromEvent } from 'rxjs';
import { auditTime, debounceTime } from 'rxjs';

const input = document.createElement('input');
input.placeholder = 'Digite palavras-chave de pesquisa';
document.body.appendChild(input);

// auditTime: Executar pesquisa a cada 300ms mesmo enquanto digita
fromEvent(input, 'input').pipe(
  auditTime(300)
).subscribe(() => {
  console.log('auditTime → Pesquisa:', input.value);
});

// debounceTime: Executar pesquisa 300ms depois que a digitação para
fromEvent(input, 'input').pipe(
  debounceTime(300)
).subscribe(() => {
  console.log('debounceTime → Pesquisa:', input.value);
});
```

### Visualização de Linha do Tempo

Quando o usuário digita "ab" → "abc" → "abcd" rapidamente:

```
Eventos de entrada:   a--b--c--d------------|
              ↓
auditTime:    ------c-----d----------|
            (após 300ms) (após 300ms)
            → Pesquisa "abc", pesquisa "abcd" (2 vezes no total)

debounceTime: --------------------d-|
                              (300ms após parar)
            → Pesquisa "abcd" (apenas 1 vez)
```

**Lembrete Fácil**:
- **`auditTime`**: "Auditar periodicamente" → Verificar em intervalos regulares
- **`debounceTime`**: "Esperar até estabelecer (debounce)" → Esperar até ficar quieto


## 🧠 Exemplo de Código Prático (Rastreamento de Mouse)

Exemplo de rastreamento de movimento do mouse e exibição da última posição em intervalos regulares.

```ts
import { fromEvent } from 'rxjs';
import { auditTime, map } from 'rxjs';

// Criar elementos UI
const container = document.createElement('div');
container.style.height = '300px';
container.style.border = '2px solid #3498db';
container.style.padding = '20px';
container.style.position = 'relative';
container.textContent = 'Mova seu mouse dentro desta área';
document.body.appendChild(container);

const positionDisplay = document.createElement('div');
positionDisplay.style.marginTop = '10px';
positionDisplay.style.fontFamily = 'monospace';
document.body.appendChild(positionDisplay);

const dot = document.createElement('div');
dot.style.width = '10px';
dot.style.height = '10px';
dot.style.borderRadius = '50%';
dot.style.backgroundColor = '#e74c3c';
dot.style.position = 'absolute';
dot.style.display = 'none';
container.appendChild(dot);

// Evento de movimento do mouse
fromEvent<MouseEvent>(container, 'mousemove').pipe(
  map(event => {
    const rect = container.getBoundingClientRect();
    return {
      x: event.clientX - rect.left,
      y: event.clientY - rect.top
    };
  }),
  auditTime(100) // Obter última posição a cada 100ms
).subscribe(position => {
  positionDisplay.textContent = `Última posição (intervalo de 100ms): X=${position.x.toFixed(0)}, Y=${position.y.toFixed(0)}`;

  // Mover ponto para a última posição
  dot.style.left = `${position.x - 5}px`;
  dot.style.top = `${position.y - 5}px`;
  dot.style.display = 'block';
});
```

Este código obtém e exibe apenas a última posição a cada 100ms, mesmo quando o mouse está se movendo com frequência.


## 🎓 Resumo

### Quando Usar auditTime
- ✅ Quando você precisa do último valor em intervalos regulares
- ✅ Eventos de alta frequência como scroll, resize, movimento do mouse
- ✅ Quando amostragem periódica é necessária
- ✅ Quando você quer refletir o último estado

### Quando Usar throttleTime
- ✅ Quando reação imediata é necessária
- ✅ Quando você quer iniciar processamento com o primeiro valor
- ✅ Prevenir pressionamento múltiplo de botão

### Quando Usar debounceTime
- ✅ Quando você quer esperar conclusão de entrada
- ✅ Pesquisa, autocompletar
- ✅ Esperar até o usuário parar de digitar

### Observações
- ⚠️ `auditTime` emite apenas o último valor dentro do período (valores intermediários são descartados)
- ⚠️ Se definido para um intervalo curto, pode não ser muito eficaz
- ⚠️ Dependendo do caso de uso, `throttleTime` ou `debounceTime` podem ser mais apropriados


## 🚀 Próximos Passos

- **[throttleTime](/pt/guide/operators/filtering/throttleTime)** - Aprenda como passar o primeiro valor
- **[debounceTime](/pt/guide/operators/filtering/debounceTime)** - Aprenda como emitir valores após a entrada parar
- **[filter](/pt/guide/operators/filtering/filter)** - Aprenda como filtrar com base em condições
- **[Exemplos Práticos de Operadores de Filtragem](/pt/guide/operators/filtering/practical-use-cases)** - Aprenda casos de uso reais
