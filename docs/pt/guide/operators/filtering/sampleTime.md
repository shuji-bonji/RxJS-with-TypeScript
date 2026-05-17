---
description: "O operador sampleTime é um operador de filtragem RxJS que coleta amostras periódicas dos valores mais recentes do fluxo em intervalos de tempo especificados. É ideal para obter instantâneos periódicos."
---

# sampleTime - obtém periodicamente o valor mais recente

O operador `sampleTime` periodicamente **mostra** o valor mais recente do Observable de origem em **intervalos de tempo especificados** e o produz.
Como um instantâneo periódico, ele recupera o valor mais recente naquele momento.

## Sintaxe básica e uso

```ts
import { fromEvent } from 'rxjs';
import { sampleTime } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.pipe(
  sampleTime(2000)
).subscribe(() => {
  console.log('2Amostras segundo a segundo');
});
```

**Fluxo de operação**:.
1. O cronômetro dispara periodicamente a cada 2 segundos
2. gera saída se houver um evento de clique recente naquele momento
3. se não houver nenhum valor durante o período de amostragem, não haverá saída

> [!WARNING] Atenção em código de produção

> O exemplo acima omite o cancelamento da assinatura de `fromEvent` para simplificar a explicação. No código real, use `takeUntil(destroy$)`, `take(N)` ou `Subscription.unsubscribe()` para gerenciar explicitamente o ciclo de vida. Mais informações: [Superando dificuldades: gerenciamento do ciclo de vida](/pt/guide/overcoming-difficulties/lifecycle-management.md)

[🌐 Documentação oficial do RxJS - `sampleTime`](https://rxjs.dev/api/operators/sampleTime)

## 💡 Padrões típicos de utilização

- Aquisição recorrente de dados de sensores**: informações atualizadas de temperatura e localização a cada segundo.
- Painel de controle em tempo real**: atualizações regulares de status
- Monitoramento de desempenho**: coleta de métricas em intervalos regulares
- Processamento de quadros de jogos**: amostragem periódica para controle de FPS

## Exemplo prático de código 1: amostragem periódica da posição do mouse

Este é um exemplo de amostragem da posição do mouse a cada segundo.

```ts
import { fromEvent } from 'rxjs';
import { sampleTime, map } from 'rxjs';

// UICriação
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Amostragem da posição do mouse (1(a cada segundo)';
container.appendChild(title);

const area = document.createElement('div');
area.style.width = '100%';
area.style.height = '300px';
area.style.border = '2px solid #4CAF50';
area.style.backgroundColor = '#f5f5f5';
area.style.display = 'flex';
area.style.alignItems = 'center';
area.style.justifyContent = 'center';
area.style.fontSize = '18px';
area.textContent = 'Mover o mouse dentro dessa área';
container.appendChild(area);

const output = document.createElement('div');
output.style.marginTop = '10px';
output.style.maxHeight = '150px';
output.style.overflow = 'auto';
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
container.appendChild(output);

let sampleCount = 0;

// Evento de movimento do mouse
fromEvent<MouseEvent>(area, 'mousemove').pipe(
  map(event => ({
    x: event.offsetX,
    y: event.offsetY,
    timestamp: Date.now()
  })),
  sampleTime(1000) // 1Amostragem a cada segundo
).subscribe(pos => {
  sampleCount++;
  const log = document.createElement('div');
  log.style.padding = '5px';
  log.style.borderBottom = '1px solid #eee';
  log.innerHTML = `
    <strong>Amostras #${sampleCount}</strong>
    [${new Date(pos.timestamp).toLocaleTimeString()}]
    Posição: (${pos.x}, ${pos.y})
  `;
  output.insertBefore(log, output.firstChild);

  // Máximo.10Exibição de até
  while (output.children.length > 10) {
    output.removeChild(output.lastChild!);
  }
});
```

- Se o mouse for movido continuamente, somente a última posição atual será amostrada a cada segundo.
- Se o mouse não for movido por um segundo, nada será gerado durante esse período.

## Exemplo prático de código 2: painel de dados em tempo real

Este exemplo mostra como os dados do sensor podem ser amostrados periodicamente e exibidos em um painel.

```ts
import { interval } from 'rxjs';
import { sampleTime, map } from 'rxjs';

// UICriação
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Painel de monitoramento do sensor';
container.appendChild(title);

const dashboard = document.createElement('div');
dashboard.style.display = 'grid';
dashboard.style.gridTemplateColumns = '1fr 1fr';
dashboard.style.gap = '10px';
dashboard.style.marginTop = '10px';
container.appendChild(dashboard);

// Criação de cartão de painel
function createCard(label: string, unit: string) {
  const card = document.createElement('div');
  card.style.padding = '20px';
  card.style.border = '2px solid #2196F3';
  card.style.borderRadius = '8px';
  card.style.backgroundColor = '#E3F2FD';

  const labelDiv = document.createElement('div');
  labelDiv.textContent = label;
  labelDiv.style.fontSize = '14px';
  labelDiv.style.color = '#666';
  card.appendChild(labelDiv);

  const valueDiv = document.createElement('div');
  valueDiv.style.fontSize = '32px';
  valueDiv.style.fontWeight = 'bold';
  valueDiv.style.marginTop = '10px';
  valueDiv.textContent = '--';
  card.appendChild(valueDiv);

  const unitDiv = document.createElement('div');
  unitDiv.textContent = unit;
  unitDiv.style.fontSize = '14px';
  unitDiv.style.color = '#666';
  card.appendChild(unitDiv);

  dashboard.appendChild(card);
  return valueDiv;
}

const tempValue = createCard('Temperatura', '°C');
const humidityValue = createCard('Umidade', '%');
const pressureValue = createCard('Pressão barométrica', 'hPa');
const lightValue = createCard('Iluminância', 'lux');

// Fluxo de dados do sensor (100msAtualizado a cada)
const sensorData$ = interval(100).pipe(
  map(() => ({
    temperature: (20 + Math.random() * 10).toFixed(1),
    humidity: (40 + Math.random() * 40).toFixed(1),
    pressure: (1000 + Math.random() * 30).toFixed(1),
    light: Math.floor(Math.random() * 1000)
  }))
);

// 2Painel de amostragem e atualização a cada segundo
sensorData$.pipe(
  sampleTime(2000)
).subscribe(data => {
  tempValue.textContent = data.temperature;
  humidityValue.textContent = data.humidity;
  pressureValue.textContent = data.pressure;
  lightValue.textContent = data.light.toString();

  // Efeito de animação
  [tempValue, humidityValue, pressureValue, lightValue].forEach(elem => {
    elem.style.color = '#2196F3';
    setTimeout(() => {
      elem.style.color = 'black';
    }, 500);
  });
});
```

- Os dados do sensor são atualizados a cada 100 ms, enquanto o painel é atualizado com valores amostrados a cada 2 segundos.
- O desempenho pode ser otimizado com a exibição de fluxos de dados de alta frequência em intervalos apropriados.

## 🆚 Comparação com operadores semelhantes

### sampleTime vs throttleTime vs auditTime

```ts
import { interval } from 'rxjs';
import { sampleTime, throttleTime, auditTime, take } from 'rxjs';

const source$ = interval(300).pipe(take(10)); // 0, 1, 2, 3, ...

// sampleTime: 1Amostragem do valor mais recente naquele momento, a cada segundo
source$.pipe(
  sampleTime(1000)
).subscribe(val => console.log('sampleTime:', val));
// Exemplos de saída: 2, 5, 8(1Instantâneo a cada segundo)

// throttleTime: Após a saída do primeiro valor,1Ignorado por 2 segundos após a saída do primeiro valor
source$.pipe(
  throttleTime(1000)
).subscribe(val => console.log('throttleTime:', val));
// Exemplos de saída: 0, 3, 6, 9(primeiro valor de cada período)

// auditTime: Saída do último valor do período1segundos após o primeiro valor, o último valor do período é emitido
source$.pipe(
  auditTime(1000)
).subscribe(val => console.log('auditTime:', val));
// Exemplos de saída: 2, 5, 8(último valor de cada período)
```

| Operador | Tempo de ignição | Valor a ser emitido | Caso de uso. |
|---|---|---|---|
| `sampleTime(1000)` | **Cronometragem recorrente a cada segundo**. | Valor mais recente naquele momento | Instantâneo periódico |
| `throttleTime(1000)` | Ignorado por 1 segundo após o recebimento do valor | Primeiro valor no início do período | Limitação de eventos |
| `auditTime(1000)` | 1 segundo após o recebimento do valor | Último valor no período | Último estado dentro do período |

**diferenças visuais**:.

```
Entrada: --|1|2|3|---|4|5|6|---|7|8|9|
      0s  1s      2s      3s

sampleTime(1s):  -------|3|-------|6|-------|9|
                 (Amostragem periódica)

throttleTime(1s): |1|--------------|4|--------------|7|
                  (Ignorado durante o período até o início)

auditTime(1s):    -------|3|-------|6|-------|9|
                  (Último valor no final do período)
```

## ⚠️ Notas.

### 1. nenhum valor durante o período da amostra

Se não houver novos valores no período de amostragem, nenhuma saída será produzida.

```ts
import { fromEvent } from 'rxjs';
import { sampleTime } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.pipe(
  sampleTime(2000)
).subscribe(() => {
  console.log('Amostras coletadas');
});
// 2Durante os segundos1Nenhuma saída se nenhum clique for feito
```

### Aguarde a primeira amostragem de tempo

O `sampleTime` não produzirá nada até que o tempo especificado tenha se passado.

```ts
import { interval } from 'rxjs';
import { sampleTime } from 'rxjs';

interval(100).pipe(
  sampleTime(1000)
).subscribe(console.log);
// O primeiro valor é1segundos após a saída do primeiro valor
```

### 3. completionTime

Quando uma fonte é concluída, a conclusão não é propagada até a próxima amostragem de tempo.

```ts
import { of } from 'rxjs';
import { sampleTime, delay } from 'rxjs';

of(1, 2, 3).pipe(
  delay(100),
  sampleTime(1000)
).subscribe({
  next: console.log,
  complete: () => console.log('Concluído')
});
// 1Segundos depois: 3
// 1Segundos depois: Concluído
```

### 4. uso de memória

A eficiência da memória é boa, pois apenas um valor mais recente é mantido internamente.

```ts
import { interval } from 'rxjs';
import { sampleTime } from 'rxjs';

// Fluxo de alta frequência (10mspor segundo)
interval(10).pipe(
  sampleTime(1000) // 1Amostragem a cada segundo
).subscribe(console.log);
// A memória retém apenas o mais recente1Apenas os dois valores mais recentes são mantidos na memória
```

## 💡 Diferenças com a amostra

O `sample` usa outro Observable como acionador, enquanto o `sampleTime` usa um intervalo de tempo fixo.

```ts
import { interval, fromEvent } from 'rxjs';
import { sample, sampleTime } from 'rxjs';

const source$ = interval(100);

// sampleTime: Intervalo de tempo fixo (1(a cada segundo)
source$.pipe(
  sampleTime(1000)
).subscribe(val => console.log('sampleTime:', val));

// sample: usando umObservableAcionado por um
const clicks$ = fromEvent(document, 'click');
source$.pipe(
  sample(clicks$)
).subscribe(val => console.log('sample:', val));
// Cada clique gera o valor mais recente naquele momento
```

| Operador | Acionadores | Caso de uso. |
|---|---|---|
| `sampleTime(ms)` | Intervalo de tempo fixo | Amostragem periódica |
| `sample(notifier$)` | Outro observável | Amostragem dinâmica de tempo |

## 📚 Operadores relacionados.

- **[sample](https://rxjs.dev/api/operators/sample)** - Amostragem de outro Observável como um acionador (documentação oficial).
- **[throttleTime](. /throttleTime)** - Obtém o primeiro valor no início do período.
- **[auditTime](. /auditTime)** - obtém o último valor no final do período
- **[debounceTime](. /debounceTime)** - emite o valor após a quiescência

## Resumo.

O operador `sampleTime` coleta periodicamente amostras do valor mais recente no intervalo de tempo especificado.

- Ideal para obter instantâneos periódicos
- Útil para reduzir os fluxos de alta frequência
- Eficiente em termos de memória (apenas um valor mais recente é retido)
- Ideal para painéis de controle e monitoramento
- ⚠️ Se nenhum valor estiver disponível durante o período de amostragem, nada será gerado
- ⚠️ Há um período de espera até a primeira amostra
- ⚠️ A conclusão é propagada no próximo período de amostragem
