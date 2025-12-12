---
description: O operador pairwise é um operador RxJS que emite dois valores consecutivos como um array de pares, e é utilizado para comparar o valor anterior com o valor atual ou para calcular a diferença.
titleTemplate: ':title | RxJS'
---

# pairwise - Processar Dois Valores Consecutivos como um Par

O operador `pairwise` **agrupa dois valores consecutivos emitidos de um stream como um array `[valor anterior, valor atual]` e os emite juntos**.
Isso é útil para comparar o valor anterior com o valor atual ou para calcular a quantidade de mudança.

## 🔰 Sintaxe Básica e Uso

```ts
import { interval } from 'rxjs';
import { pairwise, take } from 'rxjs';

interval(1000).pipe(
  take(6),
  pairwise()
).subscribe(console.log);

// Saída:
// [0, 1]
// [1, 2]
// [2, 3]
// [3, 4]
// [4, 5]
```

- O primeiro valor (0) não é emitido sozinho, mas é emitido como `[0, 1]` quando o segundo valor (1) chega.
- Sempre um par de **valor anterior e valor atual** é emitido.

[🌐 Documentação Oficial do RxJS - `pairwise`](https://rxjs.dev/api/operators/pairwise)

## 💡 Padrões de Uso Típicos

- Cálculo da quantidade de movimento do mouse ou toque
- Cálculo da quantidade de mudança (diferença) em preços ou valores
- Detecção de mudança de estado (comparação do estado anterior e estado atual)
- Determinação da direção de rolagem

## 🧠 Exemplo de Código Prático (com UI)

Este exemplo exibe a direção e quantidade de movimento do mouse.

```ts
import { fromEvent } from 'rxjs';
import { map, pairwise } from 'rxjs';

// Criar área de saída
const output = document.createElement('div');
output.style.marginTop = '10px';
output.style.fontFamily = 'monospace';
document.body.appendChild(output);

// Evento de movimento do mouse
fromEvent<MouseEvent>(document, 'mousemove').pipe(
  map(event => ({ x: event.clientX, y: event.clientY })),
  pairwise()
).subscribe(([prev, curr]) => {
  const deltaX = curr.x - prev.x;
  const deltaY = curr.y - prev.y;
  const direction = deltaX > 0 ? 'Direita' : deltaX < 0 ? 'Esquerda' : 'Parado';

  output.innerHTML = `
    Anterior: (${prev.x}, ${prev.y})<br>
    Atual: (${curr.x}, ${curr.y})<br>
    Movimento: Δx=${deltaX}, Δy=${deltaY}<br>
    Direção: ${direction}
  `;
});
```

- Quando o mouse é movido, as coordenadas anteriores e atuais e a quantidade de movimento são exibidas.
- Com `pairwise`, as coordenadas anteriores e atuais podem ser automaticamente obtidas em pares.

## 🎯 Exemplo de Cálculo da Quantidade de Mudança em um Número

Aqui está um exemplo prático de calcular a quantidade de mudança (diferença) em um stream de valores numéricos.

```ts
import { interval } from 'rxjs';
import { map, pairwise, take } from 'rxjs';

// 0, 1, 4, 9, 16, 25 (números quadrados)
interval(500).pipe(
  take(6),
  map(n => n * n),
  pairwise(),
  map(([prev, curr]) => ({
    prev,
    curr,
    diff: curr - prev
  }))
).subscribe(result => {
  console.log(`${result.prev} → ${result.curr} (diferença: +${result.diff})`);
});

// Saída:
// 0 → 1 (diferença: +1)
// 1 → 4 (diferença: +3)
// 4 → 9 (diferença: +5)
// 9 → 16 (diferença: +7)
// 16 → 25 (diferença: +9)
```

## 🎯 Determinando Direção de Rolagem

O seguinte é um exemplo de determinar a direção de rolagem (cima/baixo).

```ts
import { fromEvent } from 'rxjs';
import { map, pairwise, throttleTime } from 'rxjs';

// Criar área de saída de exibição fixa
const output = document.createElement('div');
output.style.position = 'fixed';
output.style.top = '10px';
output.style.right = '10px';
output.style.padding = '15px';
output.style.backgroundColor = 'rgba(0, 0, 0, 0.8)';
output.style.color = 'white';
output.style.fontFamily = 'monospace';
output.style.fontSize = '14px';
output.style.borderRadius = '5px';
output.style.zIndex = '9999';
document.body.appendChild(output);

// Conteúdo fictício para rolagem
const content = document.createElement('div');
content.style.height = '200vh'; // Dobrar a altura da página
content.innerHTML = '<h1>Por favor, role para baixo</h1>';
document.body.appendChild(content);

// Obter posição de rolagem
fromEvent(window, 'scroll').pipe(
  throttleTime(100), // Limitar a cada 100ms
  map(() => window.scrollY),
  pairwise()
).subscribe(([prevY, currY]) => {
  const diff = currY - prevY;
  const direction = diff > 0 ? '↓ Baixo' : '↑ Cima';
  const arrow = diff > 0 ? '⬇️' : '⬆️';

  output.innerHTML = `
    ${arrow} Direção de rolagem: ${direction}<br>
    Posição anterior: ${prevY.toFixed(0)}px<br>
    Posição atual: ${currY.toFixed(0)}px<br>
    Movimento: ${Math.abs(diff).toFixed(0)}px
  `;
});
```

- À medida que a página é rolada, a direção e informações de posição são exibidas em uma área fixa no canto superior direito.
- `pairwise` permite que você obtenha automaticamente a posição de rolagem anterior e atual em pares.

## 🎯 Utilizando pairwise Type-Safe

Este é um exemplo de utilizar a inferência de tipos do TypeScript.

```ts
import { from } from 'rxjs';
import { pairwise } from 'rxjs';

interface Stock {
  symbol: string;
  price: number;
  timestamp: number;
}

const stockPrices: Stock[] = [
  { symbol: 'AAPL', price: 150, timestamp: 1000 },
  { symbol: 'AAPL', price: 152, timestamp: 2000 },
  { symbol: 'AAPL', price: 148, timestamp: 3000 },
  { symbol: 'AAPL', price: 155, timestamp: 4000 },
];

from(stockPrices).pipe(
  pairwise()
).subscribe(([prev, curr]) => {
  const change = curr.price - prev.price;
  const changePercent = ((change / prev.price) * 100).toFixed(2);
  const trend = change > 0 ? '📈' : change < 0 ? '📉' : '➡️';

  console.log(
    `${curr.symbol}: $${prev.price} → $${curr.price} ` +
    `(${changePercent}%) ${trend}`
  );
});

// Saída:
// AAPL: $150 → $152 (1.33%) 📈
// AAPL: $152 → $148 (-2.63%) 📉
// AAPL: $148 → $155 (4.73%) 📈
```

## 🔍 Comparação com bufferCount(2, 1)

`pairwise()` é equivalente a `bufferCount(2, 1)`.

```ts
import { of } from 'rxjs';
import { pairwise, bufferCount } from 'rxjs';

const source$ = of(1, 2, 3, 4, 5);

console.log('=== pairwise ===');
source$.pipe(pairwise()).subscribe(console.log);
// Saída: [1,2], [2,3], [3,4], [4,5]

console.log('=== bufferCount(2, 1) ===');
source$.pipe(bufferCount(2, 1)).subscribe(console.log);
// Saída: [1,2], [2,3], [3,4], [4,5]
```

**Diferenças de Uso**:
- `pairwise()`: Lida explicitamente com pares de dois valores consecutivos, e a intenção do código é clara
- `bufferCount(2, 1)`: Mais flexível (pode lidar com mais de 3 tamanhos de janela)

## ⚠️ Notas

### O Primeiro Valor Não é Emitido

Como `pairwise` não emite nada até que dois valores estejam alinhados, o primeiro valor não pode ser obtido sozinho.

```ts
import { of } from 'rxjs';
import { pairwise } from 'rxjs';

of(1).pipe(pairwise()).subscribe({
  next: console.log,
  complete: () => console.log('Completado')
});

// Saída:
// Completado
// (Nenhum valor é emitido)
```

**Contramedida**: Se você quiser processar o primeiro valor também, adicione um valor inicial com `startWith`.

```ts
import { of } from 'rxjs';
import { startWith, pairwise } from 'rxjs';

of(10, 20, 30).pipe(
  startWith(0),
  pairwise()
).subscribe(console.log);

// Saída:
// [0, 10]
// [10, 20]
// [20, 30]
```

### Uso de Memória

Como `pairwise` sempre mantém apenas um valor anterior, é eficiente em termos de memória.

## 📚 Operadores Relacionados

- [`scan`](/pt/guide/operators/transformation/scan) - Processo de acumulação mais complexo
- [`bufferCount`](/pt/guide/operators/transformation/bufferCount) - Resumir valores para cada número especificado de itens
- [`distinctUntilChanged`](/pt/guide/operators/filtering/distinctUntilChanged) - Remover valores duplicados consecutivos
- [`startWith`](/pt/guide/operators/utility/startWith) - Adicionar valor inicial

## Resumo

O operador `pairwise` emite dois valores consecutivos como pares `[valor anterior, valor atual]`. Isso é muito útil para **situações onde uma comparação do valor anterior e do valor atual é necessária**, como rastrear movimentos do mouse, calcular mudanças de preços e detectar transições de estado. Note que o primeiro valor não é emitido até que o segundo valor chegue, mas isso pode ser tratado adicionando um valor inicial com `startWith`.
