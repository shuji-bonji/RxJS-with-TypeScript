---
description: Esta seção fornece uma visão geral das Creation Functions de Seleção e Partição que selecionam um Observable de vários Observables ou dividem um Observable em vários Observables. Explica como usar race e partition, bem como exemplos práticos.
---

# Creation Functions de Seleção/Partição

Estas são Creation Functions para selecionar um Observable de vários Observables ou dividir um Observable em vários Observables.

## O que são Creation Functions de Seleção/Partição?

Creation Functions de Seleção/Partição são um conjunto de funções que competem entre vários Observables para selecionar o mais rápido, ou dividem um Observable em dois streams com base em condições. Isso é útil para fontes de dados competitivas ou alocação de processamento com base em condições.

Confira a tabela abaixo para ver as características e uso de cada Creation Function.

## Principais Creation Functions de Seleção/Partição

| Função | Descrição | Casos de Uso |
|----------|------|-------------|
| **[race](/pt/guide/creation-functions/selection/race)** | Seleciona o Observable mais rápido (o que emite primeiro) | Competição entre várias fontes de dados, processamento de fallback |
| **[partition](/pt/guide/creation-functions/selection/partition)** | Divide em dois Observables com base em uma condição | Tratamento de sucesso/falha, ramificação com base em condições |

## Critérios de Uso

A seleção das Creation Functions de Seleção/Partição é determinada das seguintes perspectivas.

### 1. Propósito

- **Selecionar o mais rápido de várias fontes**: `race` - Seleciona o primeiro que emite um valor entre várias fontes de dados
- **Dividir por condição**: `partition` - Divide um Observable em dois streams com base em uma condição

### 2. Temporização de Emissão

- **Apenas o mais rápido**: `race` - Uma vez selecionado, outros valores Observable são ignorados
- **Classificar todos os valores**: `partition` - Todos os valores são classificados em dois streams de acordo com as condições

### 3. Temporização de Conclusão

- **Depende do Observable selecionado**: `race` - Segue a conclusão do Observable que emitiu primeiro
- **Depende do Observable original**: `partition` - Ambos os streams são concluídos quando o Observable original é concluído

## Exemplos de Uso Prático

### race() - Selecionar o Mais Rápido de Várias Fontes de Dados

Se você tiver várias fontes de dados e quiser usar a que responder mais rápido, use `race()`.

```typescript
import { race, timer } from 'rxjs';
import { map } from 'rxjs';

// Simular várias APIs
const api1$ = timer(1000).pipe(map(() => 'Resposta API1'));
const api2$ = timer(500).pipe(map(() => 'Resposta API2'));
const api3$ = timer(1500).pipe(map(() => 'Resposta API3'));

// Usar a resposta mais rápida
race(api1$, api2$, api3$).subscribe(console.log);
// Saída: Resposta API2 (mais rápida em 500ms)
```

### partition() - Dividir em Dois com Base em Condição

Se você quiser dividir um Observable em dois streams com base em uma condição, use `partition()`.

```typescript
import { of } from 'rxjs';
import { partition } from 'rxjs';

const numbers$ = of(1, 2, 3, 4, 5, 6, 7, 8, 9, 10);

// Dividir em números pares e ímpares
const [evens$, odds$] = partition(numbers$, n => n % 2 === 0);

evens$.subscribe(n => console.log('Par:', n));
// Saída: Par: 2, Par: 4, Par: 6, Par: 8, Par: 10

odds$.subscribe(n => console.log('Ímpar:', n));
// Saída: Ímpar: 1, Ímpar: 3, Ímpar: 5, Ímpar: 7, Ímpar: 9
```

## Convertendo Cold para Hot

Conforme mostrado na tabela acima, **todas as Creation Functions de Seleção/Partição geram Observables Cold**. A execução independente é iniciada para cada subscription.

No entanto, usando operadores multicast (`share()`, `shareReplay()`, etc.), você pode **converter um Observable Cold em um Observable Hot**.

### Exemplo Prático: Compartilhando Execução

```typescript
import { race, timer, share } from 'rxjs';
import { map } from 'rxjs';

// ❄️ Cold - Execução independente para cada subscription
const coldRace$ = race(
  timer(1000).pipe(map(() => 'API1')),
  timer(500).pipe(map(() => 'API2'))
);

coldRace$.subscribe(val => console.log('Assinante 1:', val));
coldRace$.subscribe(val => console.log('Assinante 2:', val));
// → Cada assinante executa race independente (2x requisições)

// 🔥 Hot - Compartilhar execução entre assinantes
const hotRace$ = race(
  timer(1000).pipe(map(() => 'API1')),
  timer(500).pipe(map(() => 'API2'))
).pipe(share());

hotRace$.subscribe(val => console.log('Assinante 1:', val));
hotRace$.subscribe(val => console.log('Assinante 2:', val));
// → Compartilhar execução race (requisições apenas uma vez)
```

> [!TIP]
> **Casos em que a conversão Hot é necessária**:
> - Compartilhar o resultado de `race()` entre vários componentes
> - Usar o resultado de `partition()` em vários locais
> - Executar processamento de alto custo apenas uma vez
>
> Para mais informações, consulte [Criação Básica - Convertendo Cold para Hot](/pt/guide/creation-functions/basic/#converting-cold-to-hot).

## Correspondência com Pipeable Operator

Para Creation Functions de Seleção/Partição, existe um Pipeable Operator correspondente. Quando usado em um pipeline, o operador tipo `~With` é usado.

| Creation Function | Pipeable Operator |
|-------------------|-------------------|
| `race(a$, b$)` | `a$.pipe(raceWith(b$))` |
| `partition(source$, predicate)` | Sem correspondência direta (use como Creation Function) |

> [!NOTE]
> `partition()` é tipicamente usado como uma Creation Function. Para realizar divisão de stream dentro de um pipeline, use operadores como `filter()` em combinação.

## Próximos Passos

Para aprender o comportamento detalhado e exemplos práticos de cada Creation Function, clique nos links da tabela acima.

Além disso, ao aprender [Creation Functions Básicas](/pt/guide/creation-functions/basic/), [Creation Functions de Combinação](/pt/guide/creation-functions/combination/) e [Creation Functions Condicionais](/pt/guide/creation-functions/conditional/), você pode entender o panorama geral das Creation Functions.
