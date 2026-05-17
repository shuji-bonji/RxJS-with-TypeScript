---
description: O operador sampleTime é um operador de filtragem RxJS que amostra periodicamente o último valor de um stream em intervalos de tempo especificados. É ideal para tirar snapshots periódicos.
titleTemplate: ':title | RxJS'
---

# sampleTime - Amostragem Periódica

O operador `sampleTime` **amostra periodicamente** e emite o **último valor** do Observable de origem em **intervalos de tempo especificados**.
Como snapshots periódicos, ele obtém o último valor naquele momento.

## 🔰 Sintaxe Básica e Uso

```ts
import { fromEvent } from 'rxjs';
import { sampleTime } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.pipe(
  sampleTime(2000)
).subscribe(() => {
  console.log('Amostra a cada 2 segundos');
});
```

**Fluxo de operação**:
1. Timer dispara periodicamente a cada 2 segundos
2. Se houver um último evento de clique naquele momento, emite-o
3. Se não houver valor durante o período de amostragem, nada é emitido

[🌐 Documentação Oficial RxJS - `sampleTime`](https://rxjs.dev/api/operators/sampleTime)

> [!WARNING] Atenção em código de produção
> O exemplo acima omite o cancelamento da inscrição de `fromEvent` para simplificar a explicação. Em código real, gerencie explicitamente o ciclo de vida com `takeUntil(destroy$)`, `take(N)`, ou `Subscription.unsubscribe()`. Detalhes: [Superar dificuldades: gerenciamento do ciclo de vida](/pt/guide/overcoming-difficulties/lifecycle-management.md)

## 💡 Padrões de Uso Típicos

- **Aquisição periódica de dados de sensores**: Informações de temperatura ou posição mais recentes a cada segundo
- **Dashboard em tempo real**: Atualizações periódicas de status
- **Monitoramento de desempenho**: Coleta de métricas em intervalos regulares
- **Processamento de frames de jogo**: Amostragem periódica para controle de FPS

## 🧠 Exemplo de Código Prático: Amostragem Periódica de Posição do Mouse

Exemplo de amostragem e exibição de posição do mouse a cada segundo.

```ts
import { fromEvent } from 'rxjs';
import { sampleTime, map } from 'rxjs';

// Criar UI
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Amostragem de Posição do Mouse (a cada segundo)';
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
area.textContent = 'Mova seu mouse dentro desta área';
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
  sampleTime(1000) // Amostrar a cada segundo
).subscribe(pos => {
  sampleCount++;
  const log = document.createElement('div');
  log.style.padding = '5px';
  log.style.borderBottom = '1px solid #eee';
  log.innerHTML = `
    <strong>Amostra #${sampleCount}</strong>
    [${new Date(pos.timestamp).toLocaleTimeString()}]
    Posição: (${pos.x}, ${pos.y})
  `;
  output.insertBefore(log, output.firstChild);

  // Exibir máximo 10 itens
  while (output.children.length > 10) {
    output.removeChild(output.lastChild!);
  }
});
```

- Mesmo se você continuar movendo o mouse, apenas a última posição naquele momento é amostrada a cada segundo.
- Se você não mover o mouse por 1 segundo, nada é emitido durante esse período.

## 🆚 Comparação com Operadores Similares

### sampleTime vs throttleTime vs auditTime

| Operador | Tempo de Disparo | Valor Emitido | Caso de Uso |
|:---|:---|:---|:---|
| `sampleTime(1000)` | **Tempo regular a cada 1 segundo** | Último valor naquele momento | Snapshots periódicos |
| `throttleTime(1000)` | Ignorar por 1 segundo após recepção de valor | Primeiro valor no início do período | Redução de eventos |
| `auditTime(1000)` | 1 segundo após recepção de valor | Último valor dentro do período | Último estado dentro do período |

**Diferença Visual**:

```
Entrada: --|1|2|3|---|4|5|6|---|7|8|9|
      0s  1s      2s      3s

sampleTime(1s):  -------|3|-------|6|-------|9|
                 (amostrar periodicamente)

throttleTime(1s): |1|--------------|4|--------------|7|
                  (passar primeiro e ignorar durante período)

auditTime(1s):    -------|3|-------|6|-------|9|
                  (último valor no fim do período)
```

## ⚠️ Observações

### 1. Quando Não Há Valor Durante Período de Amostragem

Se não houver novo valor no momento da amostragem, nada é emitido.

### 2. Esperar Até Primeiro Momento de Amostragem

`sampleTime` não emite nada até que o tempo especificado tenha decorrido.

### 3. Tempo de Conclusão

Mesmo se a origem completar, a conclusão não é propagada até o próximo momento de amostragem.

### 4. Uso de Memória

A eficiência de memória é boa porque mantém apenas um último valor internamente.

## 💡 Diferença de sample

`sample` usa outro Observable como trigger, enquanto `sampleTime` usa intervalos de tempo fixos.

```ts
import { interval, fromEvent } from 'rxjs';
import { sample, sampleTime } from 'rxjs';

const source$ = interval(100);

// sampleTime: Intervalo de tempo fixo (a cada 1 segundo)
source$.pipe(
  sampleTime(1000)
).subscribe(val => console.log('sampleTime:', val));

// sample: Usar outro Observable como trigger
const clicks$ = fromEvent(document, 'click');
source$.pipe(
  sample(clicks$)
).subscribe(val => console.log('sample:', val));
// Emite último valor naquele momento toda vez que você clica
```

| Operador | Trigger | Caso de Uso |
|:---|:---|:---|
| `sampleTime(ms)` | Intervalo de tempo fixo | Amostragem periódica |
| `sample(notifier$)` | Outro Observable | Amostragem em tempo dinâmico |

## 📚 Operadores Relacionados

- **[sample](https://rxjs.dev/api/operators/sample)** - Amostrar usando outro Observable como trigger (documentação oficial)
- **[throttleTime](/pt/guide/operators/filtering/throttleTime)** - Obter primeiro valor no início do período
- **[auditTime](/pt/guide/operators/filtering/auditTime)** - Obter último valor no fim do período
- **[debounceTime](/pt/guide/operators/filtering/debounceTime)** - Emitir valor após silêncio

## Resumo

O operador `sampleTime` amostra periodicamente o último valor em intervalos de tempo especificados.

- ✅ Ideal para aquisição periódica de snapshot
- ✅ Eficaz para reduzir streams de alta frequência
- ✅ Boa eficiência de memória (mantém apenas 1 último valor)
- ✅ Ideal para dashboards e monitoramento
- ⚠️ Não emite nada se não houver valor durante período de amostragem
- ⚠️ Tempo de espera até primeira amostra
- ⚠️ Conclusão propaga no próximo momento de amostragem
