---
description: "Creation Function (race e partition) que selecionam um entre vários Observable ou dividem um Observable em vários são explicadas. São apresentados casos de uso práticos e implementações type-safe no TypeScript, como tratamento de conflitos, aquisição de resposta mais rápida e particionamento de fluxo por ramificação condicional."
---

# Sistemas de seleção e particionamento Creation Function

Creation Function que selecionam um Observable entre vários Observables ou dividem um Observable em vários Observables de acordo com as condições.

## Sistema de seleção e divisão O que são Creation Function?

As Creation Functions para sistemas de seleção e divisão são diferentes daquelas para sistemas de combinação e têm as seguintes funções.

- **Select**: seleciona um Observable que satisfaz determinadas condições entre vários Observables.
- Divisão**: divide um Observable em vários Observables de acordo com uma condição.

Esses sistemas operam na direção oposta ou de uma perspectiva diferente da junção "combinar vários em um".

## Principais sistemas de seleção e divisão Creation Function

| Funções | Descrição | Caso de uso. |
|---|---|---|
| **[race](/pt/guide/creation-functions/selection/race)** | Adotar o primeiro publicado | Competição de várias fontes de dados |
| **[partition](/pt/guide/creation-functions/selection/partition)** | Dividir em dois com condições | Processo de ramificação para sucesso/falha |

## Critérios de uso

### race - selecione o Observable mais rápido

O `race` se inscreve em vários Observables simultaneamente e adota o **primeiro Observable que emite um valor. Os Observables que não são adotados são automaticamente unsubscribe.

**Caso de uso**:.
- Adotar a resposta mais rápida de vários endpoints de API
- Tratamento de tempo limite (processo original vs. cronômetro)
- Competição entre o cache e as chamadas reais de API

```typescript
import { race, timer } from 'rxjs';
import { map } from 'rxjs';

// Adotar o mais rápido de várias fontes de dados
const fast$ = timer(1000).pipe(map(() => 'Fast API'));
const slow$ = timer(3000).pipe(map(() => 'Slow API'));

race(fast$, slow$).subscribe(console.log);
// Saída.: 'Fast API' (1A saída é emitida após um segundo,slow$é cancelada)
```

### partition - dividir por condição

O comando `partition` divide um Observable em **dois Observables** com base em uma função condicional. O valor de retorno é uma matriz `[se verdadeiro, se falso]`.

**Caso de uso**:.
- Separação de sucesso e falha.
- Separação de números pares e ímpares.
- Separação de dados válidos e inválidos

```typescript
import { of, partition } from 'rxjs';

const source$ = of(1, 2, 3, 4, 5, 6);

// Dividir em números pares e ímpares
const [even$, odd$] = partition(source$, n => n % 2 === 0);

even$.subscribe(val => console.log('Even:', val));
// Saída.: Even: 2, Even: 4, Even: 6

odd$.subscribe(val => console.log('Odd:', val));
// Saída.: Odd: 1, Odd: 3, Odd: 5
```

## Conversão de frio para quente

Conforme mostrado na tabela acima, **Todas as funções de seleção e criação de sistema dividido geram um Observable Cold**. Cada assinatura inicia uma execução independente.

Os Cold Observable podem ser convertidos em Hot Observable usando operadores baseados em multicast (share()`, shareReplay()`, etc.).

### Exemplo prático: compartilhamento de solicitações de API conflitantes

```typescript
import { race, timer } from 'rxjs';
import { map, share } from 'rxjs';

// ❄️ Cold - Repetir a competição para cada assinatura
const coldRace$ = race(
  timer(1000).pipe(map(() => 'Fast API')),
  timer(3000).pipe(map(() => 'Slow API'))
);

coldRace$.subscribe(val => console.log('Assinante1:', val));
coldRace$.subscribe(val => console.log('Assinante2:', val));
// → Cada assinante realiza uma competição independente (uma2competição uma vez)

// 🔥 Hot - Compartilhar os resultados da competição entre os assinantes
const hotRace$ = race(
  timer(1000).pipe(map(() => 'Fast API')),
  timer(3000).pipe(map(() => 'Slow API'))
).pipe(share());

hotRace$.subscribe(val => console.log('Assinante1:', val));
hotRace$.subscribe(val => console.log('Assinante2:', val));
// → 1Compartilhar os resultados de uma competição
```

> [!TIP]

> Para obter mais informações, consulte [Sistema de criação básico - Conversão de frio para quente](/pt/guide/creation-functions/basic/#cold- to -hot-).

## Correspondência com o Pipeable Operator

As Creation Functions de seleção e divisão também têm um Pipeable Operator correspondente.

| Creation Function | Pipeable Operator |
|---|---|
| race(a$, b$)` | `a$.pipe(raceWith(b$))` |
| `partition(source$, predicate)` | Não pode ser usado em um pipeline (Creation Function apenas). |

> [!NOTE]

> Não existe uma versão Pipeable Operator de `partition`. Se uma partição for necessária, ela poderá ser usada como Creation Function ou dividida manualmente usando `filter` duas vezes.

## Próximas etapas.

Para saber mais sobre a operação detalhada e os exemplos práticos de cada Creation Function, clique nos links da tabela acima.

Você também pode aprender [Combination Creation Function](/pt/guide/creation-functions/combination/) e [Conditional Creation Function](/pt/guide/creation-functions/conditional/) Juntos, eles fornecem uma compreensão holística das Creation Function.
