---
description: "O operador de auditoria é um operador de filtragem RxJS que emite apenas o último valor dentro do período controlado pelo Observável personalizado. Ele é ideal para o controle dinâmico de tempo."
---

# auditoria - último valor do período de controle emitido

O operador `audit` aguarda até que um Observable personalizado emita um valor e emite o **último valor** emitido pela fonte dentro desse período.
Enquanto o `auditTime` é controlado por um tempo fixo, o `audit` permite o **controle do período** com um Observable dinâmico.

## Sintaxe básica e uso

```ts
import { fromEvent, interval } from 'rxjs';
import { audit } from 'rxjs';

// Evento de clique
const clicks$ = fromEvent(document, 'click');

// 1Períodos de tempo separados a cada segundo
clicks$.pipe(
  audit(() => interval(1000))
).subscribe(() => {
  console.log('O clique foi registrado');
});
```

- Quando ocorre um clique, inicia-se um período de um segundo.
- Somente o último clique desse período de 1 segundo é emitido.
- Após um segundo, o próximo período começa.

[🌐 Documentação oficial do RxJS - `audit`](https://rxjs.dev/api/operators/audit)

> [!WARNING] Atenção em código de produção

> O exemplo acima omite o cancelamento da assinatura do `fromEvent` para simplificar a explicação. No código real, use `takeUntil(destroy$)`, `take(N)` ou `Subscription.unsubscribe()` para gerenciar explicitamente o ciclo de vida. Mais informações: [Superando dificuldades: gerenciamento do ciclo de vida](/pt/guide/overcoming-difficulties/lifecycle-management.md)

## 💡 Padrões típicos de utilização

- Amostragem dinâmica de intervalos**: ajuste a duração de acordo com a carga.
- **Controle de tempo personalizado**: controle de período com base em outro observável.
- Limitação adaptativa de eventos**: redução sensível ao contexto

## 🔍 Diferenças com o auditTime

| Operador. | Controle de período | Caso de uso. |
|---|---|---|
| `auditTime`. | Tempo fixo (milissegundos) | Controle simples baseado em tempo |
| `audit`. | **Observável personalizado** | **Controle dinâmico de período**. |

```ts
import { fromEvent, timer } from 'rxjs';
import { audit, auditTime } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// auditTime - Fixo1segundos
clicks$.pipe(
  auditTime(1000)
).subscribe(() => console.log('Fixo1segundos'));

// audit - Período dinâmico
let period = 1000;
clicks$.pipe(
  audit(() => {
    period = Math.random() * 2000; // 0~.2Período aleatório de segundos
    return timer(period);
  })
).subscribe(() => console.log(`Período dinâmico: ${period}ms`));
```

## Exemplo prático de código 1: amostragem dinâmica baseada em carga

Este é um exemplo de ajuste do intervalo de amostragem de acordo com a carga do sistema.

```ts
import { fromEvent, timer } from 'rxjs';
import { audit, map } from 'rxjs';

// UICriação
const output = document.createElement('div');
output.innerHTML = '<h3>Amostragem dinâmica</h3>';
document.body.appendChild(output);

const button = document.createElement('button');
button.textContent = 'Mudança de carga';
document.body.appendChild(button);

const statusDiv = document.createElement('div');
statusDiv.style.marginTop = '10px';
output.appendChild(statusDiv);

const logDiv = document.createElement('div');
logDiv.style.marginTop = '10px';
logDiv.style.maxHeight = '200px';
logDiv.style.overflow = 'auto';
output.appendChild(logDiv);

// Nível de carga (0: Carga baixa,1: Carga média,2: Carga alta)
let loadLevel = 0;

fromEvent(button, 'click').subscribe(() => {
  loadLevel = (loadLevel + 1) % 3;
  const levels = ['Carga baixa', 'Carga média', 'Carga alta'];
  statusDiv.textContent = `Carga atual: ${levels[loadLevel]}`;
});

// Evento de movimento do mouse
const moves$ = fromEvent<MouseEvent>(document, 'mousemove');

moves$.pipe(
  audit(() => {
    // Duração dependendo da carga
    const periods = [2000, 1000, 500]; // Carga baixa→Longa duração, carga alta→Curta duração
    return timer(periods[loadLevel]);
  }),
  map(event => ({ x: event.clientX, y: event.clientY }))
).subscribe(pos => {
  const log = document.createElement('div');
  log.textContent = `[${new Date().toLocaleTimeString()}] Posição do mouse: (${pos.x}, ${pos.y})`;
  logDiv.insertBefore(log, logDiv.firstChild);

  // Máximo.10Exibição até
  while (logDiv.children.length > 10) {
    logDiv.removeChild(logDiv.lastChild!);
  }
});
```

- Reduzido para intervalos de 2 s quando a carga é baixa (modo de economia de energia)
- Amostragem fina em intervalos de 500 ms quando a carga é alta.
- O período pode ser ajustado dinamicamente de acordo com a carga.

## Exemplo prático de código 2: Controle de período com base em outros fluxos

```ts
import { fromEvent, timer } from 'rxjs';
import { audit, map, startWith } from 'rxjs';

// UICriação
const container = document.createElement('div');
document.body.appendChild(container);

const slider = document.createElement('input');
slider.type = 'range';
slider.min = '100';
slider.max = '2000';
slider.value = '1000';
container.appendChild(document.createTextNode('Intervalo: '));
container.appendChild(slider);

const intervalDisplay = document.createElement('span');
intervalDisplay.textContent = ' 1000ms';
container.appendChild(intervalDisplay);

const output = document.createElement('div');
output.style.marginTop = '10px';
container.appendChild(output);

// Monitorar valores do controle deslizante
const sliderValue$ = fromEvent(slider, 'input').pipe(
  map(() => Number(slider.value)),
  startWith(1000)
);

sliderValue$.subscribe(value => {
  intervalDisplay.textContent = ` ${value}ms`;
});

// Evento de clique
const clicks$ = fromEvent(document, 'click');

let currentInterval = 1000;

// Atualizar os valores do controle deslizante
sliderValue$.subscribe(value => {
  currentInterval = value;
});

// Clique emauditControlado por
clicks$.pipe(
  audit(() => timer(currentInterval))
).subscribe(() => {
  const log = document.createElement('div');
  log.textContent = `[${new Date().toLocaleTimeString()}] Clique em registro (intervalo: ${currentInterval}ms)`;
  output.insertBefore(log, output.firstChild);
});
```

## ⚠️ Notas.

### 1. O primeiro valor não é emitido imediatamente

Depois que o `audit` recebe o primeiro valor, ele aguarda até o final do período.

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
// 9  (1Segundos depois,0~.9Último valor de)
// 19 (2Segundos depois,10~.19Último valor de)
// 29 (3Segundos depois,20~.29Último valor de)
```

### O Observável de duração é gerado novamente a cada vez.

As funções passadas para `audit` **devem retornar um novo Observável a cada vez**.

```ts
// ❌ Exemplo ruim: Se a mesmaObservableinstância for usada e usada novamente
const duration$ = timer(1000);
source$.pipe(
  audit(() => duration$) // 2Não está funcionando após a segunda vez
).subscribe();

// ✅ Bom exemplo: Um novoObservableGera um
source$.pipe(
  audit(() => timer(1000))
).subscribe();
```

### 3. memória e desempenho

O uso do `audit` em fluxos em que os valores são emitidos consome memória com frequência.

```ts
import { interval, timer } from 'rxjs';
import { audit } from 'rxjs';

// fluxo rápido (10mspor segundo)
interval(10).pipe(
  audit(() => timer(1000)) // 1Amostragem a cada segundo
).subscribe();
// 1por segundo100Os valores são armazenados na memória e somente o último é emitido1Apenas o último é emitido
```

## 🆚 Comparação com operadores semelhantes

| Operadores | Quando emitir | Valor a ser emitido | Caso de uso. |
|---|---|---|---|
| `audit`. | No **fim** do período | O **último** valor dentro do período | Obter o status mais recente dentro do período |
| `throttle`. | No **início** do período | Valor do **primeiro** no período | Obtém o início de uma sequência de eventos. |
| `debounce`. | **After** stationary**. | Valor imediatamente anterior ao estacionário | Aguardar a conclusão da entrada |
| `sample`. | **Quando outro observável for acionado**. | Valor mais recente no momento | Instantâneo periódico |

```ts
import { fromEvent, interval, timer } from 'rxjs';
import { audit, throttle, debounce, sample } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// audit: 1Último clique em segundos
clicks$.pipe(
  audit(() => timer(1000))
).subscribe(() => console.log('audit: Último clique'));

// throttle: 1Primeiro clique em segundos
clicks$.pipe(
  throttle(() => timer(1000))
).subscribe(() => console.log('throttle: Primeiro clique'));

// debounce: Depois que o clique parar1Segundos após
clicks$.pipe(
  debounce(() => timer(1000))
).subscribe(() => console.log('debounce: Após a parada'));

// sample: 1Amostragem a cada segundo
clicks$.pipe(
  sample(interval(1000))
).subscribe(() => console.log('sample: Periódico'));
```

## 📚 Operadores relacionados.

- **[auditTime](. /auditTime)** - controlado por tempo fixo (versão simplificada do `audit`).
- **[throttle](. /throttleTime)** - primeiro valor emitido no início do período.
- **[debounce](. /debounceTime)** - emite um valor após um período de inatividade.
- **[sample](. /sampleTime)** - amostra no tempo de outro Observável

## Resumo.

O operador `audit` emite o último valor em um período controlado dinamicamente por um Observable personalizado.

- É possível o controle dinâmico do período.
- Amostragem adaptável com base na carga
- Controle baseado em outros fluxos
- ⚠️ Um novo Observable precisa ser gerado a cada vez
- ⚠️ Sensível à memória para emissão frequente
