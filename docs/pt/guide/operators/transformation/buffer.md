---
description: buffer é um operador RxJS que gera um array de valores acumulados no momento em que outro Observable emite um valor, tornando-o ideal para processamento em lote orientado a eventos.
titleTemplate: ':title | RxJS'
---

# buffer - Coleta Valores no Momento de Outro Observable

O operador `buffer` acumula os valores de um Observable fonte **até** que outro Observable emita um valor, e então gera os valores acumulados como um **array** nesse momento.
Isso é útil quando você deseja controlar o buffering de acordo com eventos ou sinais externos, em vez de por tempo ou número de itens.

## 🔰 Sintaxe Básica e Uso

```ts
import { interval, fromEvent } from 'rxjs';
import { buffer } from 'rxjs';

// Emite valores a cada 100ms
const source$ = interval(100);

// Usa evento de clique como gatilho
const clicks$ = fromEvent(document, 'click');

source$.pipe(
  buffer(clicks$)
).subscribe(bufferedValues => {
  console.log('Valores acumulados até o clique:', bufferedValues);
});

// Exemplo de saída (gera a cada clique):
// Valores acumulados até o clique: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
// Valores acumulados até o clique: [11, 12, 13, 14, 15, 16, 17]
// ...
```

- Cada vez que `clicks$` emite um valor, os valores acumulados até esse ponto são gerados como um array.
- A característica é que a delimitação do buffer pode ser controlada por um Observable externo.

[🌐 Documentação Oficial RxJS - `buffer`](https://rxjs.dev/api/operators/buffer)

## 💡 Padrões Típicos de Uso

- Processamento em lote acionado por ações do usuário
- Coleta e transmissão de dados baseados em sinais externos
- Agrupamento de eventos com delimitação dinâmica
- Envio em lote quando conexão WebSocket ou API é estabelecida

## 🔍 Diferença de bufferTime / bufferCount

| Operador | Momento da Delimitação | Uso |
|:---|:---|:---|
| `buffer` | **Outro Observable emite** | Controle orientado a eventos |
| `bufferTime` | **Intervalo de tempo fixo** | Processamento em lote baseado em tempo |
| `bufferCount` | **Contagem fixa** | Processamento em lote baseado em contagem |

```ts
import { interval, timer } from 'rxjs';
import { buffer } from 'rxjs';

const source$ = interval(100);
// Gatilho a cada 1 segundo
const trigger$ = timer(1000, 1000);

source$.pipe(
  buffer(trigger$)
).subscribe(values => {
  console.log('Valores a cada segundo:', values);
});

// Saída:
// Valores a cada segundo: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
// Valores a cada segundo: [10, 11, 12, 13, 14, 15, 16, 17, 18, 19]
```

## 🧠 Exemplo de Código Prático (com UI)

Este é um exemplo de acionar um clique de botão e registrar todos os eventos de movimento do mouse até esse ponto juntos.

```ts
import { fromEvent } from 'rxjs';
import { map, buffer } from 'rxjs';

// Criar botão e área de saída
const button = document.createElement('button');
button.textContent = 'Registrar Movimento do Mouse';
document.body.appendChild(button);

const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Evento de movimento do mouse
const mouseMoves$ = fromEvent<MouseEvent>(document, 'mousemove').pipe(
  map(event => ({ x: event.clientX, y: event.clientY }))
);

// Gatilho ao clicar no botão
const clicks$ = fromEvent(button, 'click');

mouseMoves$.pipe(
  buffer(clicks$)
).subscribe(positions => {
  const message = `Eventos detectados: ${positions.length} itens`;
  console.log(message);
  console.log('Dados de coordenadas:', positions.slice(0, 5)); // Exibe apenas os primeiros 5
  output.textContent = message;
});
```

- Todos os movimentos do mouse até o clique do botão são armazenados em um buffer.
- Como os eventos são processados juntos no momento do clique, o processamento em lote em momento arbitrário é possível.

## 🎯 Exemplo Avançado com Múltiplos Gatilhos

Controle mais flexível é possível combinando vários Observables de gatilho.

```ts
import { interval, merge, fromEvent, timer } from 'rxjs';
import { buffer, mapTo } from 'rxjs'; } from 'rxjs';

const source$ = interval(100);

// Múltiplos gatilhos: clique ou 5 segundos decorridos
const clicks$ = fromEvent(document, 'click').pipe(map(() => 'click'));
const fiveSeconds$ = timer(5000, 5000).pipe(map(() => 'timer'));
const trigger$ = merge(clicks$, fiveSeconds$);

source$.pipe(
  buffer(trigger$)
).subscribe(values => {
  console.log(`Saída do buffer (${values.length} itens):`, values);
});
```

## ⚠️ Notas

### Cuidado com Vazamentos de Memória

Como `buffer` continua acumulando valores até o próximo gatilho, pode consumir memória excessiva se um gatilho não ocorrer por muito tempo.

```ts
// Exemplo ruim: Gatilho pode não ocorrer
const neverTrigger$ = fromEvent(document.querySelector('.non-existent'), 'click');

source$.pipe(
  buffer(neverTrigger$) // Gatilho nunca ocorre, buffer acumula infinitamente
).subscribe();
```

**Contramedidas**:
- Limitar o tamanho máximo do buffer em combinação com `bufferTime` e `bufferCount`
- Adicionar tratamento de timeout

```ts
import { interval, fromEvent, timer, race } from 'rxjs';
import { buffer } from 'rxjs';

const source$ = interval(100);

// Múltiplos gatilhos: clique ou 5 segundos decorridos
const clicks$ = fromEvent(document, 'click');
const timeout$ = timer(10000); // Timeout após máximo de 10 segundos

source$.pipe(
  buffer(race(clicks$, timeout$)) // Emite no que vier primeiro
).subscribe(values => {
  console.log('Buffer:', values);
});
```

## 📚 Operadores Relacionados

- [`bufferTime`](/pt/guide/operators/transformation/bufferTime) - Buffering baseado em tempo
- [`bufferCount`](/pt/guide/operators/transformation/bufferCount) - Buffering baseado em contagem
- [`bufferToggle`](https://rxjs.dev/api/operators/bufferToggle) - Controle de buffering com Observable de início e fim
- [`bufferWhen`](https://rxjs.dev/api/operators/bufferWhen) - Buffering com condições de fechamento dinâmicas
- [`window`](/pt/guide/operators/transformation/windowTime) - Retorna Observable em vez de buffer

## Resumo

O operador `buffer` é uma ferramenta poderosa para processar um lote de valores acionados por um Observable externo. Ele permite processamento em lote **orientado a eventos**, em vez de tempo ou número de itens. No entanto, cuidado com vazamentos de memória quando os gatilhos não ocorrem.
