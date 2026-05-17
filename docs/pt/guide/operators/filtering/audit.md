---
description: O operador audit é um operador de filtragem RxJS que emite apenas o último valor dentro de um período controlado por um Observable customizado. É ideal para controle de tempo dinâmico.
titleTemplate: ':title'
---

# audit - Ultimo valor em trigger

O operador `audit` espera por um Observable customizado emitir um valor e emite o **último valor** da fonte durante esse período.
Enquanto `auditTime` controla com um tempo fixo, `audit` pode **controlar o período dinamicamente com um Observable**.

## 🔰 Sintaxe Básica e Uso

```ts
import { fromEvent, interval } from 'rxjs';
import { audit } from 'rxjs';

// Evento de clique
const clicks$ = fromEvent(document, 'click');

// Separar período a cada segundo
clicks$.pipe(
  audit(() => interval(1000))
).subscribe(() => {
  console.log('Clique foi registrado');
});
```

- Quando um clique ocorre, um período de 1 segundo começa.
- Apenas o último clique durante esse 1 segundo é emitido.
- O próximo período começa após 1 segundo.

[🌐 Documentação Oficial RxJS - `audit`](https://rxjs.dev/api/operators/audit)

> [!WARNING] Atenção em código de produção
> O exemplo acima omite o cancelamento da inscrição de `fromEvent` para simplificar a explicação. Em código real, gerencie explicitamente o ciclo de vida com `takeUntil(destroy$)`, `take(N)`, ou `Subscription.unsubscribe()`. Detalhes: [Superar dificuldades: gerenciamento do ciclo de vida](/pt/guide/overcoming-difficulties/lifecycle-management.md)

## 💡 Padrões de Uso Típicos

- **Amostragem em intervalos dinâmicos**: Ajustar período de acordo com a carga
- **Controle de tempo customizado**: Controle de período baseado em outros Observables
- **Limitação adaptativa de eventos**: Redução de acordo com as circunstâncias

## 🔍 Diferença de auditTime

| Operador | Controle de Período | Caso de Uso |
|:---|:---|:---|
| `auditTime` | Tempo fixo (milissegundos) | Controle simples baseado em tempo |
| `audit` | **Observable Customizado** | **Controle de período dinâmico** |

```ts
import { fromEvent, timer } from 'rxjs';
import { audit, auditTime } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// auditTime - 1 segundo fixo
clicks$.pipe(
  auditTime(1000)
).subscribe(() => console.log('1 segundo fixo'));

// audit - Período dinâmico
let period = 1000;
clicks$.pipe(
  audit(() => {
    period = Math.random() * 2000; // Período aleatório 0-2 segundos
    return timer(period);
  })
).subscribe(() => console.log(`Período dinâmico: ${period}ms`));
```

## 🧠 Exemplo de Código Prático: Amostragem Dinâmica De Acordo com a Carga

Exemplo de ajuste de intervalo de amostragem de acordo com a carga do sistema.

```ts
import { fromEvent, timer } from 'rxjs';
import { audit, map } from 'rxjs';

// Criar UI
const output = document.createElement('div');
output.innerHTML = '<h3>Amostragem Dinâmica</h3>';
document.body.appendChild(output);

const button = document.createElement('button');
button.textContent = 'Mudar Carga';
document.body.appendChild(button);

const statusDiv = document.createElement('div');
statusDiv.style.marginTop = '10px';
output.appendChild(statusDiv);

const logDiv = document.createElement('div');
logDiv.style.marginTop = '10px';
logDiv.style.maxHeight = '200px';
logDiv.style.overflow = 'auto';
output.appendChild(logDiv);

// Nível de carga (0: baixo, 1: médio, 2: alto)
let loadLevel = 0;

fromEvent(button, 'click').subscribe(() => {
  loadLevel = (loadLevel + 1) % 3;
  const levels = ['Carga Baixa', 'Carga Média', 'Carga Alta'];
  statusDiv.textContent = `Carga atual: ${levels[loadLevel]}`;
});

// Evento de movimento do mouse
const moves$ = fromEvent<MouseEvent>(document, 'mousemove');

moves$.pipe(
  audit(() => {
    // Ajustar período de acordo com a carga
    const periods = [2000, 1000, 500]; // Carga baixa → período longo, carga alta → período curto
    return timer(periods[loadLevel]);
  }),
  map(event => ({ x: event.clientX, y: event.clientY }))
).subscribe(pos => {
  const log = document.createElement('div');
  log.textContent = `[${new Date().toLocaleTimeString()}] Posição do mouse: (${pos.x}, ${pos.y})`;
  logDiv.insertBefore(log, logDiv.firstChild);

  // Exibir máximo 10 itens
  while (logDiv.children.length > 10) {
    logDiv.removeChild(logDiv.lastChild!);
  }
});
```

- Quando a carga é baixa, reduz em intervalos de 2 segundos (modo de economia de energia)
- Quando a carga é alta, amostra finamente em intervalos de 500ms
- Período pode ser ajustado dinamicamente de acordo com a carga


## ⚠️ Observações

### 1. Primeiro Valor Não é Emitido Imediatamente

`audit` espera até o período terminar após receber o primeiro valor.

```ts
import { interval, timer } from 'rxjs';
import { audit, take } from 'rxjs';

interval(100).pipe(
  audit(() => timer(1000)),
  take(3)
).subscribe(val => {
  console.log(val);
});
// Saída:
// 9  (após 1 segundo, último valor de 0-9)
// 19 (após 2 segundos, último valor de 10-19)
// 29 (após 3 segundos, último valor de 20-29)
```

### 2. Observable de Duração Deve Ser Gerado Novamente a Cada Vez

A função passada para `audit` deve **retornar um novo Observable a cada vez**.

```ts
// ❌ Exemplo ruim: Reutilizar mesma instância de Observable
const duration$ = timer(1000);
source$.pipe(
  audit(() => duration$) // Não funciona após a 2ª vez
).subscribe();

// ✅ Bom exemplo: Gerar novo Observable a cada vez
source$.pipe(
  audit(() => timer(1000))
).subscribe();
```

## 🆚 Comparação com Operadores Similares

| Operador | Tempo de Emissão | Valor Emitido | Caso de Uso |
|:---|:---|:---|:---|
| `audit` | No **fim** do período | **Último** valor dentro do período | Obter último estado dentro do período |
| `throttle` | No **início** do período | **Primeiro** valor dentro do período | Obter primeiro de eventos consecutivos |
| `debounce` | **Após silêncio** | Valor antes do silêncio | Esperar conclusão de entrada |
| `sample` | **Quando outro Observable dispara** | Último valor naquele momento | Snapshots periódicos |


## 📚 Operadores Relacionados

- **[auditTime](/pt/guide/operators/filtering/auditTime)** - Controlar com tempo fixo (versão simplificada de `audit`)
- **[throttle](/pt/guide/operators/filtering/throttleTime)** - Emitir primeiro valor no início do período
- **[debounce](/pt/guide/operators/filtering/debounceTime)** - Emitir valor após silêncio
- **[sample](/pt/guide/operators/filtering/sampleTime)** - Amostrar no tempo de outro Observable

## Resumo

O operador `audit` emite o último valor dentro de um período controlado dinamicamente por um Observable customizado.

- ✅ Controle de período dinâmico possível
- ✅ Amostragem adaptativa de acordo com a carga
- ✅ Controle baseado em outros streams
- ⚠️ Deve gerar novo Observable a cada vez
- ⚠️ Esteja atento à memória com emissões frequentes
