---
description: windowCount é um operador de conversão do RxJS que divide um Observable por um número especificado de itens. É ideal para processamento de streams baseado em contagem, agregação por contagem fixa e processamento de paginação. Diferentemente de bufferCount, pode aplicar processamento independente a cada janela. A inferência de tipos do TypeScript permite divisão de janelas type-safe e operações de stream.
titleTemplate: ':title | RxJS'
---

# windowCount - Dividir Observable por Contagem Especificada

O operador `windowCount` **divide** valores emitidos em novos Observables para cada contagem especificada.
Enquanto `bufferCount` retorna um array, `windowCount` retorna um **Observable&lt;T&gt;**, permitindo que operadores adicionais sejam aplicados a cada janela.

## 🔰 Sintaxe Básica e Uso

```ts
import { interval } from 'rxjs';
import { windowCount, mergeAll } from 'rxjs';

// Emitir valores a cada 100ms
const source$ = interval(100);

source$.pipe(
  windowCount(5),
  mergeAll() // Achatar cada janela
).subscribe(value => {
  console.log('Valor na janela:', value);
});

// Saída:
// Valor na janela: 0
// Valor na janela: 1
// Valor na janela: 2
// Valor na janela: 3
// Valor na janela: 4
// (Nova janela começa)
// Valor na janela: 5
// ...
```

- Uma nova janela (Observable) é criada a cada 5 valores.
- É único no sentido de que divide com base em contagem.

[🌐 Documentação Oficial do RxJS - `windowCount`](https://rxjs.dev/api/operators/windowCount)

## 💡 Padrões de Uso Típicos

- Processamento de agregação para cada contagem fixa
- Transmissão em lote de dados (processamento diferente para cada janela)
- Processamento de paginação
- Calcular estatísticas por janela

## 🔍 Diferença em relação a bufferCount

| Operador | Saída | Caso de Uso |
|:---|:---|:---|
| `bufferCount` | **Array (T[])** | Processar valores agrupados juntos |
| `windowCount` | **Observable&lt;T&gt;** | Processamento de stream diferente para cada grupo |

```ts
import { interval } from 'rxjs';
import { bufferCount, windowCount, mergeAll } from 'rxjs';

const source$ = interval(100);

// bufferCount - Saída como array
source$.pipe(
  bufferCount(5)
).subscribe(values => {
  console.log('Buffer (array):', values);
  // Saída: Buffer (array): [0, 1, 2, 3, 4]
});

// windowCount - Saída como Observable
source$.pipe(
  windowCount(5)
).subscribe(window$ => {
  console.log('Window (Observable):', window$);
  window$.subscribe(value => {
    console.log('  Valor na janela:', value);
  });
});
```

## 🧠 Exemplo de Código Prático 1: Soma por Janela

Este é um exemplo de cálculo da soma de cada 5 valores.

```ts
import { interval } from 'rxjs';
import { windowCount, map, mergeAll, reduce } from 'rxjs';

// Criar área de saída
const output = document.createElement('div');
output.innerHTML = '<h3>Soma a Cada 5 Valores</h3>';
document.body.appendChild(output);

const source$ = interval(200);

let windowNumber = 0;

source$.pipe(
  windowCount(5),
  map(window$ => {
    const current = ++windowNumber;
    console.log(`Janela ${current} iniciada`);

    // Calcular soma para cada janela
    return window$.pipe(
      reduce((sum, value) => sum + value, 0),
      map(sum => ({ windowNum: current, sum }))  // Incluir número da janela
    );
  }),
  mergeAll()
).subscribe(result => {
  const div = document.createElement('div');
  div.textContent = `Janela ${result.windowNum} soma: ${result.sum}`;
  output.appendChild(div);
});

// Saída:
// Janela 1 soma: 10  (0+1+2+3+4)
// Janela 2 soma: 35  (5+6+7+8+9)
// Janela 3 soma: 60  (10+11+12+13+14)
```

## 🎯 Exemplo de Código Prático 2: Especificando Índice Inicial

Você pode especificar um índice inicial com o segundo argumento. Isso cria janelas sobrepostas.

```ts
import { range } from 'rxjs';
import { windowCount, mergeMap, toArray } from 'rxjs';

// Emitir valores de 0 a 9
range(0, 10).pipe(
  windowCount(3, 2), // 3 itens cada, início deslocado por 2
  mergeMap(window$ => window$.pipe(toArray()))
).subscribe(values => {
  console.log('Janela:', values);
});

// Saída:
// Janela: [0, 1, 2]
// Janela: [2, 3, 4]    ← Iniciado deslocado por 2 (de 2)
// Janela: [4, 5, 6]    ← Iniciado deslocado por 2 (de 4)
// Janela: [6, 7, 8]
// Janela: [8, 9]       ← Últimos 2 itens
```

### Padrões de Operação do Índice Inicial

```ts
// windowCount(bufferSize, startBufferEvery)

windowCount(3, 3) // Contínuo (padrão): [0,1,2], [3,4,5], [6,7,8]
windowCount(3, 2) // Sobreposição: [0,1,2], [2,3,4], [4,5,6]
windowCount(3, 4) // Com intervalo: [0,1,2], [4,5,6], [8,9,10]
```

## 🎯 Exemplo Prático: Processamento Diferente para Cada Janela

```ts
import { interval } from 'rxjs';
import { windowCount, map, mergeAll, take } from 'rxjs';

const source$ = interval(100);
let windowNumber = 0;

source$.pipe(
  windowCount(5),
  map(window$ => {
    const current = ++windowNumber;

    if (current % 2 === 0) {
      // Janelas pares: Obter apenas os primeiros 2 itens
      console.log(`Janela ${current}: Obter primeiros 2 itens`);
      return window$.pipe(take(2));
    } else {
      // Janelas ímpares: Obter todos
      console.log(`Janela ${current}: Obter todos`);
      return window$;
    }
  }),
  mergeAll()
).subscribe(value => {
  console.log(`Valor: ${value} (Janela ${windowNumber})`);
});
```

## 🧠 Exemplo de Código Prático 3: Processamento Tipo Paginação

```ts
import { from } from 'rxjs';
import { windowCount, mergeMap, toArray, map } from 'rxjs';

// Dados de 1 a 20
const data$ = from(Array.from({ length: 20 }, (_, i) => i + 1));

// Paginar por 5 itens
data$.pipe(
  windowCount(5),
  mergeMap((window$, index) => {
    const pageNumber = index + 1;
    return window$.pipe(
      toArray(),
      map(items => ({ page: pageNumber, items }))
    );
  })
).subscribe(page => {
  console.log(`Página ${page.page}:`, page.items);
});

// Saída:
// Página 1: [1, 2, 3, 4, 5]
// Página 2: [6, 7, 8, 9, 10]
// Página 3: [11, 12, 13, 14, 15]
// Página 4: [16, 17, 18, 19, 20]
```

## ⚠️ Notas

### 1. Gerenciamento de Subscrição de Janela

Cada janela é um Observable independente e deve ser explicitamente subscrita.

```ts
source$.pipe(
  windowCount(5)
).subscribe(window$ => {
  // Valores não fluirão a menos que você se inscreva na janela em si
  window$.subscribe(value => {
    console.log('Valor:', value);
  });
});
```

Ou use `mergeAll()`, `concatAll()`, `switchAll()`, etc. para achatar.

### 2. Última Janela

Ao completar o Observable de origem, a última janela é emitida mesmo que contenha menos do que o número especificado de itens.

```ts
import { of } from 'rxjs';
import { windowCount, mergeMap, toArray } from 'rxjs';

of(1, 2, 3, 4, 5, 6, 7).pipe(
  windowCount(3),
  mergeMap(window$ => window$.pipe(toArray()))
).subscribe(values => {
  console.log('Janela:', values);
});

// Saída:
// Janela: [1, 2, 3]
// Janela: [4, 5, 6]
// Janela: [7]  ← Apenas 1 item
```

### 3. Uso de Memória por Índice Inicial

Se `startBufferEvery` for menor que `bufferSize` (sobreposição), várias janelas estarão ativas ao mesmo tempo, aumentando o uso de memória.

```ts
// Sobreposição: Máximo de 2 janelas ativas simultaneamente
windowCount(5, 3)

// Contramedida: Limitar com take() se necessário
source$.pipe(
  take(100), // Máximo de 100 itens
  windowCount(5, 3)
)
```

## 🆚 Comparação de Operadores window

| Operador | Momento do Delimitador | Caso de Uso |
|:---|:---|:---|
| `window` | Outro Observable emite | Particionamento orientado a eventos |
| `windowTime` | Intervalo de tempo fixo | Particionamento baseado em tempo |
| `windowCount` | **Contagem fixa** | **Particionamento baseado em contagem** |
| `windowToggle` | Observables de início e fim | Controle dinâmico de início/fim |
| `windowWhen` | Condição de fechamento dinâmica | Condição de fim diferente por janela |

## 📚 Operadores Relacionados

- [`bufferCount`](/pt/guide/operators/transformation/bufferCount) - Coletar valores como array (versão array de windowCount)
- [`window`](/pt/guide/operators/transformation/window) - Dividir janela em diferentes momentos do Observable
- [`windowTime`](/pt/guide/operators/transformation/windowTime) - Divisão de janela baseada em tempo
- [`windowToggle`](/pt/guide/operators/transformation/windowToggle) - Controle de janela com Observables de início e fim
- [`windowWhen`](/pt/guide/operators/transformation/windowWhen) - Divisão de janela com condições de fechamento dinâmicas

## Resumo

O operador `windowCount` é uma ferramenta útil para particionar streams com base em contagem e tratar cada grupo como um Observable independente.

- ✅ Ideal para agregação e processamento por contagem fixa
- ✅ Processamento diferente pode ser aplicado a cada janela
- ✅ Pode ser sobreposto por índice inicial
- ⚠️ Requer gerenciamento de subscrição
- ⚠️ Esteja ciente do uso de memória ao sobrepor
