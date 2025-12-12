---
description: reduce é um operador de conversão do RxJS que acumula todos os valores em um stream e emite apenas o resultado final ao concluir. É ideal para situações onde apenas o resultado de agregação final é necessário, como calcular somas, médias, máximos, mínimos, agregar objetos e construir arrays. Diferentemente de scan, não emite resultados intermediários e não pode ser usado com streams infinitos porque a conclusão do stream é necessária.
titleTemplate: ':title | RxJS'
---

# reduce - Emite Apenas o Resultado Acumulado Final

O operador `reduce` aplica uma função cumulativa a cada valor no stream e emite **apenas o resultado cumulativo final** na conclusão do stream.
Funciona da mesma forma que `Array.prototype.reduce` para arrays, sem emitir resultados intermediários.

## 🔰 Sintaxe Básica e Uso

```ts
import { of } from 'rxjs';
import { reduce } from 'rxjs';

of(1, 2, 3, 4, 5)
  .pipe(reduce((acc, curr) => acc + curr, 0))
  .subscribe(console.log);

// Saída: 15 (apenas resultado final)
```

- `acc` é o valor cumulativo, `curr` é o valor atual.
- Os valores são acumulados sequencialmente, começando pelo valor inicial (`0` neste caso).
- Nenhum valor é emitido até que o stream seja concluído, e **apenas o resultado final** é emitido na conclusão.

[🌐 Documentação Oficial do RxJS - `reduce`](https://rxjs.dev/api/operators/reduce)

## 💡 Padrões de Uso Típicos

- Calcular somas, médias, máximos e mínimos de números
- Agregar e transformar objetos
- Construir ou combinar arrays
- Quando apenas o resultado de agregação final é necessário

## 🔍 Diferença em relação a scan

| Operador | Momento da Emissão | Conteúdo da Emissão | Uso |
|:---|:---|:---|:---|
| `reduce` | **Apenas uma vez na conclusão** | Resultado cumulativo final | Agregação onde apenas o resultado final é necessário |
| `scan` | **Toda vez para cada valor** | Todos incluindo resultados intermediários | Agregação em tempo real/gerenciamento de estado |

```ts
import { of } from 'rxjs';
import { reduce, scan } from 'rxjs';

const source$ = of(1, 2, 3, 4, 5);

console.log('=== reduce ===');
source$.pipe(
  reduce((acc, curr) => acc + curr, 0)
).subscribe(console.log);
// Saída: 15

console.log('=== scan ===');
source$.pipe(
  scan((acc, curr) => acc + curr, 0)
).subscribe(console.log);
// Saída: 1, 3, 6, 10, 15
```

## 🧠 Exemplo de Código Prático (com UI)

Este exemplo soma os valores de vários campos de entrada e exibe o resultado final ao clicar em um botão.

```ts
import { fromEvent, from } from 'rxjs';
import { map, reduce, switchMap } from 'rxjs';

// Criar campos de entrada
const inputs: HTMLInputElement[] = [];
for (let i = 1; i <= 3; i++) {
  const label = document.createElement('label');
  label.textContent = `Valor ${i}: `;
  const input = document.createElement('input');
  input.type = 'number';
  input.value = '0';
  label.appendChild(input);
  document.body.appendChild(label);
  document.body.appendChild(document.createElement('br'));
  inputs.push(input);
}

// Botão de calcular
const button = document.createElement('button');
button.textContent = 'Calcular Soma';
document.body.appendChild(button);

// Área de exibição do resultado
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Calcular soma ao clicar no botão
fromEvent(button, 'click').pipe(
  switchMap(() => {
    // Obter todos os valores de entrada
    const values = inputs.map(input => Number(input.value) || 0);
    return from(values).pipe(
      reduce((acc, curr) => acc + curr, 0)
    );
  })
).subscribe(total => {
  output.textContent = `Total: ${total}`;
  console.log('Total:', total);
});
```

- Ao clicar no botão, todos os valores de entrada são somados e apenas o total final é exibido.
- Resultados intermediários não são emitidos.

## 🎯 Exemplo de Agregação de Objetos

Este é um exemplo prático de agregar múltiplos valores em um objeto.

```ts
import { from } from 'rxjs';
import { reduce } from 'rxjs';

interface Product {
  category: string;
  price: number;
}

const products: Product[] = [
  { category: 'Comida', price: 500 },
  { category: 'Bebida', price: 200 },
  { category: 'Comida', price: 800 },
  { category: 'Bebida', price: 150 },
  { category: 'Comida', price: 300 },
];

// Agregar preço total por categoria
from(products).pipe(
  reduce((acc, product) => {
    acc[product.category] = (acc[product.category] || 0) + product.price;
    return acc;
  }, {} as Record<string, number>)
).subscribe(result => {
  console.log('Total por categoria:', result);
});

// Saída:
// Total por categoria: { Comida: 1600, Bebida: 350 }
```

## 🎯 Exemplo de Construção de Array

Aqui está um exemplo de combinar valores de stream em um array.

```ts
import { interval } from 'rxjs';
import { take, reduce } from 'rxjs';

interval(100).pipe(
  take(5),
  reduce((acc, value) => {
    acc.push(value);
    return acc;
  }, [] as number[])
).subscribe(array => {
  console.log('Array coletado:', array);
});

// Saída:
// Array coletado: [0, 1, 2, 3, 4]
```

::: tip
Ao construir um array, considere usar o operador [`toArray`](/pt/guide/operators/utility/toArray) mais conciso.
```ts
interval(100).pipe(
  take(5),
  toArray()
).subscribe(console.log);
// Saída: [0, 1, 2, 3, 4]
```
:::

## 💡 Utilizando reduce Type-Safe

Aqui está um exemplo de utilizar a inferência de tipos do TypeScript.

```ts
import { from } from 'rxjs';
import { reduce } from 'rxjs';

interface UserAction {
  type: 'click' | 'scroll' | 'input';
  timestamp: number;
}

const actions: UserAction[] = [
  { type: 'click', timestamp: 100 },
  { type: 'scroll', timestamp: 200 },
  { type: 'click', timestamp: 300 },
  { type: 'input', timestamp: 400 },
];

const actions$ = from(actions);

// Agregar contagem por tipo de ação
actions$.pipe(
  reduce((acc, action) => {
    acc[action.type] = (acc[action.type] || 0) + 1;
    return acc;
  }, {} as Record<UserAction['type'], number>)
).subscribe(result => {
  console.log('Agregação de ações:', result);
});

// Saída:
// Agregação de ações: { click: 2, scroll: 1, input: 1 }
```

## ⚠️ Notas

### ❌ Streams Infinitos Não Completam (Importante)

> [!WARNING]
> **`reduce` não emitirá um único valor até que `complete()` seja chamado.** Streams infinitos (`interval`, `fromEvent`, etc.) causam acidentes na prática, já que nenhum valor fica permanentemente disponível.

```ts
import { interval } from 'rxjs';
import { reduce } from 'rxjs';

// ❌ Exemplo ruim: Stream infinito, então nenhum valor é emitido
interval(1000).pipe(
  reduce((acc, curr) => acc + curr, 0)
).subscribe(console.log);
// Sem saída (stream não completa)
```

**Contramedida 1: Use `scan` quando agregação contínua é necessária**

```ts
import { interval, scan, take } from 'rxjs';

// ✅ Bom exemplo: Obter resultados intermediários em tempo real
interval(1000).pipe(
  take(5),
  scan((acc, curr) => acc + curr, 0)
).subscribe(console.log);
// Saída: 0, 1, 3, 6, 10 (emite valor cumulativo toda vez)
```

**Contramedida 2: Se apenas o valor final é necessário, use `scan` + `takeLast(1)`**

```ts
import { interval, scan, take, takeLast } from 'rxjs';

// ✅ Bom exemplo: Acumular com scan, obter apenas valor final
interval(1000).pipe(
  take(5),
  scan((acc, curr) => acc + curr, 0),
  takeLast(1)
).subscribe(console.log);
// Saída: 10 (apenas resultado final)
```

**Contramedida 3: Use `take` para especificar a condição de término**

```ts
import { interval, take, reduce } from 'rxjs';

// ✅ Bom exemplo: Definir condição de término com take
interval(1000).pipe(
  take(5),
  reduce((acc, curr) => acc + curr, 0)
).subscribe(console.log);
// Saída: 10
```

> [!TIP]
> **Critérios de Seleção**:
> - Resultados intermediários são necessários → `scan`
> - Apenas o resultado final é necessário & conclusão do stream é garantida → `reduce`
> - Apenas resultado final necessário & stream infinito → `scan` + `takeLast(1)` ou `take` + `reduce`

### Uso de Memória

Quando o valor cumulativo é um objeto grande ou array, o uso de memória deve ser levado em conta.

```ts
// Exemplo requerendo atenção à memória
from(largeDataArray).pipe(
  reduce((acc, item) => {
    acc.push(item); // Acumular grandes quantidades de dados
    return acc;
  }, [])
).subscribe();
```

## 📚 Operadores Relacionados

- [`scan`](/pt/guide/operators/transformation/scan) - Emite um resultado intermediário para cada valor
- [`toArray`](/pt/guide/operators/utility/toArray) - Combinar todos os valores em um array
- [`count`](https://rxjs.dev/api/operators/count) - Conta o número de valores
- [`min`](https://rxjs.dev/api/operators/min) / [`max`](https://rxjs.dev/api/operators/max) - Obter valores mínimo e máximo

## Resumo

O operador `reduce` acumula todos os valores em um stream e emite **apenas o resultado final na conclusão**. Isso é adequado quando resultados intermediários não são necessários e apenas o resultado de agregação final é necessário. No entanto, como nenhum resultado é obtido se o stream não for concluído, você deve usar `scan` para streams infinitos ou definir uma condição de saída com `take` ou similar.
