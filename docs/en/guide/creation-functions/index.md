---
description: A comprehensive explanation of RxJS Creation Functions (Observable creation functions), including differences from Pipeable Operator, basic usage, and seven categories (basic creation, loop generation, HTTP communication, combination, selection/partition, conditional branching, and control systems).
---

# Creation Functions

In RxJS, there are two different forms: **Creation Functions** for creating Observables and **Pipeable Operators** for converting existing Observables.

This page describes the basic concepts of Creation Functions and the seven main categories.

## What are Creation Functions?

**Creation Functions** are functions for creating new Observables.

```typescript
import { of, from, interval } from 'rxjs';

// Using as Creation Functions
const obs1$ = of(1, 2, 3);
const obs2$ = from([4, 5, 6]);
const obs3$ = interval(1000);
```

They are imported directly from the `rxjs` package and called as functions to create Observables.

## Difference from Pipeable Operator

Creation Functions and Pipeable Operators have different uses and applications. See the table below to see the differences between them.

| Feature | Creation Function | Pipeable Operator |
|------|-------------------|-------------------|
| **Purpose** | Create new Observable | Transform existing Observable |
| **Import from** | `rxjs` | `rxjs/operators` |
| **Usage** | Call directly as a function | Use inside `.pipe()` |
| **Example** | `concat(obs1$, obs2$)` | `obs1$.pipe(concatWith(obs2$))` |

### Example of Creation Function

Creation Functions are used to directly combine multiple Observables.

```typescript
import { concat, of } from 'rxjs';

const obs1$ = of(1, 2, 3);
const obs2$ = of(4, 5, 6);

// Using as Creation Function
concat(obs1$, obs2$).subscribe(console.log);
// Output: 1, 2, 3, 4, 5, 6
```

### Example of Pipeable Operator

The Pipeable Operator is used to add a conversion process to an existing Observable.

```typescript
import { of } from 'rxjs';
import { concatWith } from 'rxjs';

const obs1$ = of(1, 2, 3);
const obs2$ = of(4, 5, 6);

// Using as Pipeable Operator
obs1$.pipe(
  concatWith(obs2$)
).subscribe(console.log);
// Output: 1, 2, 3, 4, 5, 6
```

## Usage Criteria

The choice between Creation Function and Pipeable Operator is determined by the following criteria.

### When Creation Function Should Be Used

The Creation Function is suitable when multiple Observables are to be operated at the same level or when an Observable is to be created from scratch.

- **When combining multiple Observables at the same level**
  ```typescript
  concat(obs1$, obs2$, obs3$)
  merge(click$, hover$, scroll$)
  ```

- **When creating an Observable from scratch**
  ```typescript
  of(1, 2, 3)
  from([1, 2, 3])
  interval(1000)
  ```

### When Pipeable Operator Should Be Used

The Pipeable Operator is suitable for adding processing to an existing Observable or for chaining multiple operations together.

- **When adding operations to an existing Observable**
  ```typescript
  obs1$.pipe(
    map(x => x * 2),
    concatWith(obs2$),
    filter(x => x > 5)
  )
  ```

- **When chaining multiple operations as a pipeline**

## Categories of Creation Functions

In this chapter, Creation Functions are divided into seven categories.

### List of All Categories

In the table below, you can see all categories and the functions they contain. Click on each function name to go to the detail page.

| Category | Description | Main Functions | Typical Use Cases |
|---------|------|-----------|-------------------|
| **[Basic Creation](/en/guide/creation-functions/basic/)** | Most basic and frequently used functions. Create data, array, event, and time-based Observables | [of](/en/guide/creation-functions/basic/of), [from](/en/guide/creation-functions/basic/from), [fromEvent](/en/guide/creation-functions/basic/fromEvent), [interval](/en/guide/creation-functions/basic/interval), [timer](/en/guide/creation-functions/basic/timer) | Testing with fixed values, streaming existing data, DOM event handling, polling, delayed execution |
| **[Loop Generation](/en/guide/creation-functions/loop/)** | Express loop processing like for/while statements in Observable | [range](/en/guide/creation-functions/loop/range), [generate](/en/guide/creation-functions/loop/generate) | Sequential number generation, batch processing, complex state transitions, mathematical calculations |
| **[HTTP Communication](/en/guide/creation-functions/http-communication/)** | Handle HTTP communication as Observable | [ajax](/en/guide/creation-functions/http-communication/ajax), [fromFetch](/en/guide/creation-functions/http-communication/fromFetch) | XMLHttpRequest-based HTTP communication, Fetch API-based HTTP communication, REST API calls |
| **[Combination](/en/guide/creation-functions/combination/)** | Combine multiple Observables into one. Emission timing and order differ depending on the combination method | [concat](/en/guide/creation-functions/combination/concat), [merge](/en/guide/creation-functions/combination/merge), [combineLatest](/en/guide/creation-functions/combination/combineLatest), [zip](/en/guide/creation-functions/combination/zip), [forkJoin](/en/guide/creation-functions/combination/forkJoin) | Step-by-step processing, integration of multiple events, synchronization of form inputs, waiting for completion of parallel API calls |
| **[Selection/Partition](/en/guide/creation-functions/selection/)** | Select one from multiple Observables or partition one Observable into multiple | [race](/en/guide/creation-functions/selection/race), [partition](/en/guide/creation-functions/selection/partition) | Competition between multiple data sources, success/failure branching |
| **[Conditional](/en/guide/creation-functions/conditional/)** | Select Observable based on conditions or dynamically generate at subscription time | [iif](/en/guide/creation-functions/conditional/iif), [defer](/en/guide/creation-functions/conditional/defer) | Processing branching based on login status, dynamic Observable creation, lazy evaluation |
| **[Control](/en/guide/creation-functions/control/)** | Control Observable execution timing and resource management | [scheduled](/en/guide/creation-functions/control/scheduled), [using](/en/guide/creation-functions/control/using) | Execution timing control with scheduler, resource lifecycle management, memory leak prevention |

> [!TIP]
> **Learning Order**
>
> We recommend that beginners learn in the following order:
> 1. **Basic Creation** - Fundamental RxJS functions
> 2. **Combination** - Basics of handling multiple streams
> 3. **HTTP Communication** - Practical API integration
> 4. Other categories - Learn as needed

## Correspondence with Pipeable Operator

Many Creation Functions have a corresponding Pipeable Operator. When used in a pipeline, use an operator of the `~With` family.

| Creation Function | Pipeable Operator | Notes |
|-------------------|-------------------|------|
| `concat(a$, b$)` | `a$.pipe(`**[concatWith](/en/guide/operators/combination/concatWith)**`(b$))` | RxJS 7+ |
| `merge(a$, b$)` | `a$.pipe(`**[mergeWith](/en/guide/operators/combination/mergeWith)**`(b$))` | RxJS 7+ |
| `zip(a$, b$)` | `a$.pipe(`**[zipWith](/en/guide/operators/combination/zipWith)**`(b$))` | RxJS 7+ |
| `combineLatest([a$, b$])` | `a$.pipe(`**[combineLatestWith](/en/guide/operators/combination/combineLatestWith)**`(b$))` | RxJS 7+ |
| `race(a$, b$)` | `a$.pipe(`**[raceWith](/en/guide/operators/combination/raceWith)**`(b$))` | RxJS 7+ |

> [!NOTE]
> Since RxJS 7, **[concatWith](/en/guide/operators/combination/concatWith)**, **[mergeWith](/en/guide/operators/combination/mergeWith)**, **[zipWith](/en/guide/operators/combination/zipWith)**, **[combineLatestWith](/en/guide/operators/combination/combineLatestWith)**, **[raceWith](/en/guide/operators/combination/raceWith)** and other `~With` type operators have been added, making it easier to use as Pipeable Operator.

## Which Should I Use?

The choice between Creation Function and Pipeable Operator depends on the context.

### Creation Function is Recommended

If multiple Observables are to be operated at the same level, the Creation Function will simplify the code.

```typescript
// ✅ Combine multiple Observables at the same level
const combined$ = merge(
  fromEvent(button1, 'click'),
  fromEvent(button2, 'click'),
  fromEvent(button3, 'click')
);
```

### Pipeable Operator is Recommended

When adding operations as part of a pipeline, use Pipeable Operator to clarify the flow of processing.

```typescript
// ✅ Combine as part of pipeline
const result$ = source$.pipe(
  map(x => x * 2),
  mergeWith(other$),
  filter(x => x > 10)
);
```

## Summary

- **Creation Functions**: Functions to create and combine Observables
- **Pipeable Operators**: Functions to convert existing Observables
- Creation Functions fall into 7 categories:
  1. **Basic Creation**: Create data, array, event, and time-based Observables
  2. **Loop Generation**: Express iterative processing in Observable
  3. **HTTP Communication**: Handle HTTP communication as Observable
  4. **Combination**: Combine multiple into one
  5. **Selection/Partition**: Select or partition
  6. **Conditional**: Generate dynamically according to conditions
  7. **Control**: Control execution timing and resource management
- Use `~With` family Pipeable Operators in pipelines
- Each category contains multiple functions and can be used in different ways depending on the application

## Next Steps

To learn more about each category, please follow the links below:

1. **[Basic Creation Functions](/en/guide/creation-functions/basic/)** - of, from, fromEvent, interval, timer
2. **[Loop Generation Functions](/en/guide/creation-functions/loop/)** - range, generate
3. **[HTTP Communication Functions](/en/guide/creation-functions/http-communication/)** - ajax, fromFetch
4. **[Combination Functions](/en/guide/creation-functions/combination/)** - concat, merge, combineLatest, zip, forkJoin
5. **[Selection/Partition Functions](/en/guide/creation-functions/selection/)** - race, partition
6. **[Conditional Functions](/en/guide/creation-functions/conditional/)** - iif, defer
7. **[Control Functions](/en/guide/creation-functions/control/)** - scheduled, using

On each page, you will learn more about how Creation Functions work and practical examples.

## Reference Resources

- [RxJS Official Documentation - Creation Functions](https://rxjs.dev/guide/operators#creation-operators-list)
- [Learn RxJS - Creation Operators](https://www.learnrxjs.io/learn-rxjs/operators/creation)
