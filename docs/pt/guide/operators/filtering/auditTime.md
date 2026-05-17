---
description: "auditTime é um operador de filtragem RxJS que espera por um tempo especificado quando um valor é emitido e gera o último valor dentro desse período. É melhor usado quando se deseja obter amostras periódicas do estado mais recente em eventos de alta frequência, como rastreamento da posição de rolagem, redimensionamento da janela, movimento do mouse etc. É importante entender a diferença entre esse operador e o throttleTime e o debounceTime e usá-los adequadamente."
---

# AuditTime - último valor emitido após o tempo especificado

O operador auditTime aguarda um **tempo especificado** após a emissão de um valor e gera o **último valor** dentro desse período. Em seguida, ele aguarda a chegada do próximo valor.

## Sintaxe básica e uso

```ts
import { fromEvent } from 'rxjs';
import { auditTime } from 'rxjs';

fromEvent(document, 'click').pipe(
  auditTime(1000)
).subscribe(() => console.log('Clique em.！'));
```

**Fluxo de operação**:.
1. ocorre o primeiro clique
2. espera por 1 segundo (os cliques durante esse tempo são registrados, mas não são emitidos)
3. Emite o último clique após 1 segundo
Aguardar o próximo clique

[🌐 Documentação oficial do RxJS - auditTime](https://rxjs.dev/api/operators/auditTime)

## 🆚 Contraste com throttleTime

O `throttleTime` e o `auditTime` são semelhantes, mas diferem nos valores que produzem.

```ts
import { interval } from 'rxjs';
import { throttleTime, auditTime, take } from 'rxjs';

const source$ = interval(300).pipe(take(10)); // 0, 1, 2, 3, 4, 5, 6, 7, 8, 9

// throttleTime: Saída do primeiro valor
source$.pipe(
  throttleTime(1000)
).subscribe(console.log);
// Saída.: 0, 4, 8(primeiro valor de cada período)

// auditTime: Saída do último valor
source$.pipe(
  auditTime(1000)
).subscribe(console.log);
// Saída.: 3, 6, 9(último valor de cada período)
```

**Comparação de linha do tempo**:.

```
Fonte:     0--1--2--3--4--5--6--7--8--9--|
            |        |        |
throttle:   0--------4--------8------------|
            (Primeiro)   (Primeiro)   (Primeiro)

audit:      -------3--------6--------9----|
                  (Último)   (Último)   (Último)
```

```ts
import { fromEvent } from 'rxjs';
import { auditTime } from 'rxjs';

fromEvent(document, 'click').pipe(
  auditTime(1000)
).subscribe(() => console.log('Clique em.！'));
```ts
import { interval } from 'rxjs';
import { throttleTime, auditTime, take } from 'rxjs';

const source$ = interval(300).pipe(take(10)); // 0, 1, 2, 3, 4, 5, 6, 7, 8, 9

// throttleTime: PrimeiroのvalorをSaída
source$.pipe(
  throttleTime(1000)
).subscribe(console.log);
// Saída: 0, 4, 8（各períodoのPrimeiroのvalor）

// auditTime: ÚltimoのvalorをSaída
source$.pipe(
  auditTime(1000)
).subscribe(console.log);
// Saída: 3, 6, 9（各períodoのÚltimoのvalor）
```

## 💡 Padrão de utilização típico

1. **Otimizar o redimensionamento da janela**.

```ts
   import { fromEvent } from 'rxjs';
   import { auditTime } from 'rxjs';

   fromEvent(window, 'resize').pipe(
     auditTime(200) // 200msObter o último tamanho no intervalo
   ).subscribe(() => {
     console.log(`Tamanho da janela: ${window.innerWidth}x${window.innerHeight}`);
   });
   ```

2. **Rastreamento da posição de rolagem**
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

3. **Movimento de arrasto suave**
   ```ts
   import { fromEvent } from 'rxjs';
   import { auditTime, map, takeUntil, switchMap } from 'rxjs';

   // Criar elementos arrastáveis
   const box = document.createElement('div');
   box.style.width = '100px';
   box.style.height = '100px';
   box.style.backgroundColor = '#3498db';
   box.style.position = 'absolute';
   box.style.cursor = 'move';
   box.style.left = '100px';
   box.style.top = '100px';
   box.textContent = 'Arrastar';
   box.style.display = 'flex';
   box.style.alignItems = 'center';
   box.style.justifyContent = 'center';
   box.style.color = 'white';
   document.body.appendChild(box);

   const mouseDown$ = fromEvent<MouseEvent>(box, 'mousedown');
   const mouseMove$ = fromEvent<MouseEvent>(document, 'mousemove');
   const mouseUp$ = fromEvent<MouseEvent>(document, 'mouseup');

   // Implementar operações de arrastar
   mouseDown$.pipe(
     switchMap(startEvent => {
       const startX = startEvent.clientX - box.offsetLeft;
       const startY = startEvent.clientY - box.offsetTop;

       return mouseMove$.pipe(
         auditTime(16), // Aprox.60FPS(consulte também16ms) para atualizar a posição
         map(moveEvent => ({
           x: moveEvent.clientX - startX,
           y: moveEvent.clientY - startY
         })),
         takeUntil(mouseUp$)
       );
     })
   ).subscribe(position => {
     box.style.left = `${position.x}px`;
     box.style.top = `${position.y}px`;
   });
   ```

## 🧠 Exemplo prático de código (rastreamento do mouse)

Este exemplo rastreia os movimentos do mouse e exibe a posição mais recente em intervalos regulares.

```

ts.
import { fromEvent } from 'rxjs';
import { auditTime, map } from 'rxjs';

// Criação de elementos da interface do usuário
const container = document.createElement('div');.
container.style.height = '300px';
contêiner.style.border = '2px solid #3498db';
padding do contêiner = '20px';
contentor.style.position = 'relative';
container.textContent = 'Por favor, mova o mouse dentro desta área';
document.body.appendChild(contêiner);

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
contêiner.appendChild(dot);

// Evento de movimentação do mouse
fromEvent<MouseEvent>(container, 'mousemove').pipe(
  map(event => {
    const rect = container.getBoundingClientRect();
    return {
      x: event.clientX - rect.left,.
      y: event.clientY - rect.top
    };
  }),
  auditTime(100) // Obtém a posição mais recente a cada 100ms
).subscribe(position => {
  positionDisplay.textContent = `Latest position (every 100ms): X=${position.x.toFixed(0)}, Y=${position.y.toFixed(0)}`;

  // Mover o ponto para a última posição
  dot.style.left = `${position.x - 5}px`;
  dot.style.top = `${position.y - 5}px`;
  dot.style.display = 'block';
});

```

Esse código só recuperará e exibirá a posição mais recente sempre que o mouse for movido, mesmo que o mouse seja movido com frequência,100msO código só recupera e exibe a posição mais recente para cada movimento do mouse.

## 🎯 debounceTime Diferenças entre

`auditTime` e `debounceTime` é que**ambos geram o último valor**mas o**O tempo é completamente diferente**o último valor é emitido.

### A diferença decisiva

| Operador | operação | uso do sistema de maneiras diferentes |
|---|---|---|
| `auditTime(ms)` | Quando um valor é recebido**msSempre emite após**(mesmo que a entrada continue) | Amostragem periódica |
| `debounceTime(ms)` | **Depois que a entrada for interrompida**msSaída depois | Aguardar a conclusão da entrada |

### Exemplos específicos：Diferenças na entrada de pesquisa

```

ts.
import { fromEvent } from 'rxjs';
import { auditTime, debounceTime } from 'rxjs';

const input = document.createElement('input');
input.placeholder = 'Search word input';
document.body.appendChild(input);

// AuditTime: executa a pesquisa a cada 300ms, mesmo durante a entrada de dados
fromEvent(input, 'input').pipe(
  auditTime(300)
).subscribe(() => {
  console.log('auditTime → Pesquisa:', input.value);
});

// debounceTime: aguarde 300 ms após a parada da entrada e, em seguida, execute a pesquisa
fromEvent(input, 'input').pipe(
  debounceTime(300)
).subscribe(() => {
  console.log('debounceTime → Pesquisa:', input.value);
});

```

### Diferenças observadas na linha do tempo

Diferença observada quando um usuário clica em "ab'→'abc'→'abcd' ao digitar rapidamente:

```

Evento de entrada: a--b--c--d------------|
              ↓
auditTime: ------c-----d----------|
            (após 300 ms) (após 300 ms)
            → Pesquisar por 'abc', pesquisar por 'abcd' (2 vezes no total)

debounceTime: --------------------d-|
                              (300 ms após a parada)
            → Pesquisar por "abcd" (apenas uma vez no total)

```

**Fácil de lembrar**:
- **`auditTime`**: 'Regularmente auditado (audit)"→ 'Sempre verifique em intervalos regulares'
- **`debounceTime`**: 'Wait until it has settled down (...)'.debounceAguardar até que esteja calmo.→ 'Espere até que esteja calmo'

### Uso prático

```

ts.
// ✅ auditTime, se apropriado
// - Rastreamento da posição de rolagem (queremos obtê-la periodicamente, mesmo se estivermos rolando o tempo todo)
fromEvent(window, 'scroll').pipe(
  auditTime(100) // obtém a posição mais recente a cada 100ms
).subscribe(/* ... */);

// ✅ se o debounceTime for apropriado.
// - caixa de pesquisa (queremos pesquisar depois que a entrada for concluída)
fromEvent(searchInput, 'input').pipe(
  debounceTime(300) // esperar 300 ms após a parada da entrada
).subscribe(/* ... */);

```

## 📋 Uso com segurança de tipo

TypeScript Este é um exemplo de uma implementação com segurança de tipo que faz uso de genéricos em

```

ts.
import { Observable, fromEvent } from 'rxjs';
import { auditTime, map } from 'rxjs';

interface MousePosition {
  x: número;
  y: number;
  timestamp: number; }
}

function trackMousePosition(
  element: HTMLElement,.
  intervalMs: number
): Observable {
  return fromEvent<MouseEvent>(element, 'mousemove').pipe(
    auditTime(intervalMs),.
    map(event => ({
      x: event.clientX, event.
      y: event.clientY,.
      timestamp: Date.now())
    } as MousePosition))
  );
}

// Exemplo de uso
const canvas = document.createElement('div');
canvas.style.width = '400px';
canvas.style.height = '300px';
canvas.style.border = '1px solid black';
document.body.appendChild(canvas);

trackMousePosition(canvas, 200).subscribe(position => {
  console.log(`Posição: (${position.x}, ${position.y}) em ${position.timestamp}`);
});

```

## 🔄 auditTime e throttleTime Combinação de

Em determinados cenários, ambos podem ser combinados.

```

ts.
import { interval } from 'rxjs';
import { throttleTime, auditTime, take } from 'rxjs';

const source$ = interval(100).pipe(take(50));.

// ordem de throttleTime → auditTime
source$.pipe(
  throttleTime(1000), // passa o primeiro valor a cada segundo
  auditTime(500) // então aguarde 500 ms e envie o último valor
).subscribe(console.log);.

```ts
import { fromEvent } from 'rxjs';
import { auditTime } from 'rxjs';

fromEvent(document, 'click').pipe(
  auditTime(1000)
).subscribe(() => console.log('Clique em.！'));
---
description: auditTimeは値が発行されたら指定時間待機し、その期間内の最後の値を出力するRxJSフィルタリングオペレーターです。スクロール位置の追跡、ウィンドウリサイズ、マウス移動などの高頻度イベントで最新の状態を定期的にサンプリングしたい場合に最適です。throttleTimeやdebounceTimeとの違いを理解して適切に使い分けることが重要です。
---


ts.
import { fromEvent } from 'rxjs';
import { auditTime } from 'rxjs';

// Criar um campo de entrada de pesquisa
const input = document.createElement('input');.
input.type = 'text';
input.placeholder = 'Search...' ;
document.body.appendChild(input);

// ❌ Exemplo ruim: use auditTime para a entrada de pesquisa
fromEvent(input, 'input').pipe(
  auditTime(300) // a pesquisa é realizada a cada 300ms durante a entrada
).subscribe(() => {
  console.log('Pesquisa executada');
});

```ts
import { fromEvent } from 'rxjs';
import { auditTime } from 'rxjs';

fromEvent(document, 'click').pipe(
  auditTime(1000)
).subscribe(() => console.log('Clique em.！'));
```ts
import { fromEvent } from 'rxjs';
import { auditTime } from 'rxjs';

fromEvent(document, 'click').pipe(
  auditTime(1000)
).subscribe(() => console.log('クリック！'));
```

ts.
import { fromEvent } from 'rxjs';
import { debounceTime } from 'rxjs';

// Criar um campo de entrada de pesquisa
const input = document.createElement('input');.
input.type = 'text';
input.placeholder = 'Search...' ;
document.body.appendChild(input);

// ✅ Bom exemplo: usar debounceTime para a entrada de pesquisa
fromEvent(input, 'input').pipe(
  debounceTime(300) // Aguarde 300 ms após a parada da entrada antes de pesquisar
).subscribe(() => {
  console.log('Pesquisa executada', input.value);
});
```

## 🎓 Resumo

### Quando o auditTime deve ser usado.
- ✅ Quando valores atualizados são necessários em intervalos regulares
- Eventos de alta frequência, como rolagem, redimensionamento, movimento do mouse
- Quando for necessária uma amostragem periódica
- Quando você quiser refletir o estado mais recente.

### Quando o throttleTime deve ser usado.
- Quando for necessária uma resposta imediata
- Se você quiser iniciar o processamento com o primeiro valor
- Prevenção de pressionamento de botão

### Quando usar o debounceTime.
- Se você quiser aguardar a conclusão da entrada
- Pesquisa, autocompletar
- Aguarde até que o usuário pare de digitar.

### Notas.
- ⚠️ O auditTime gera apenas o último valor do período (os valores intermediários são descartados).
- ⚠️ Não é muito eficaz se for definido para intervalos curtos.
- ⚠️ O `throttleTime` ou o `debounceTime` podem ser mais apropriados, dependendo do aplicativo.

## 🚀 Próximas etapas.

- **[throttleTime](. /throttleTime)** - aprender como passar o primeiro valor.
- **[debounceTime](. /debounceTime)** - aprenda a emitir valores após a parada da entrada.
- **[filter](. /filter)** - aprenda a filtrar com base em condições.
- **[filtering-operator-practical-use-cases](. /practical-use-cases)** - aprenda a usar casos de uso reais
