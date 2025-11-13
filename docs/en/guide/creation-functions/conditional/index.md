---
description: This section describes Creation Functions that select and create Observables based on conditions. Learn how to use iif and defer, as well as practical examples.
---

# Conditional Creation Functions

Creation Functions select an Observable based on a condition or dynamically generate an Observable when subscribing.

## What are Conditional Creation Functions?

Conditional Creation Functions have the following roles:

- **Conditional Selection**: Select different Observables according to conditions
- **Delayed Generation**: Dynamically create an Observable upon subscription

Unlike other Creation Functions, which statically create and combine Observables, these can change their behavior based on **run-time conditions and states**.

> [!NOTE]
> Although `iif` and `defer` were previously classified as "conditional operators", they are **Creation Functions** (Observable creation functions), not Pipeable Operators.

## Major Conditional Creation Functions

| Function | Description | Use Cases |
|----------|------|-------------|
| **[iif](/en/guide/creation-functions/conditional/iif)** | Select one of two Observables based on a condition | Processing branching based on login status |
| **[defer](/en/guide/creation-functions/conditional/defer)** | Delay generation of Observable at subscription time | Dynamic Observable creation |

## Usage Criteria

### iif - Two Branches Based on Condition

`iif` selects one of two Observables depending on the result of a conditional function. The condition is evaluated **at subscription time**.

**Syntax**:
```typescript
iif(
  () => condition,  // Condition function (evaluated at subscription time)
  trueObservable,   // Observable if true
  falseObservable   // Observable if false
)
```

**Use Cases**:
- Process branching based on login status
- Switching processing based on whether cache exists
- Behavior change by environment variables

```typescript
import { iif, of } from 'rxjs';

const isAuthenticated = () => Math.random() > 0.5;

const data$ = iif(
  isAuthenticated,
  of('Authenticated data'),
  of('Public data')
);

data$.subscribe(console.log);
// Output: 'Authenticated data' or 'Public data' (depending on condition at subscription time)
```

### defer - Delayed Generation at Subscription Time

`defer` generates an Observable each time a subscription occurs. This allows the Observable to change its behavior based on its state at the time of subscription.

**Syntax**:
```typescript
defer(() => {
  // Executed at subscription time
  return someObservable;
})
```

**Use Cases**:
- Generate Observable reflecting the latest state at the time of subscription
- Generate a different random value each time
- Perform different processing for each subscription

```typescript
import { defer, of } from 'rxjs';

// Get current time at subscription
const timestamp$ = defer(() => of(new Date().toISOString()));

setTimeout(() => {
  timestamp$.subscribe(time => console.log('First:', time));
}, 1000);

setTimeout(() => {
  timestamp$.subscribe(time => console.log('Second:', time));
}, 2000);

// Output:
// First: 2024-10-21T01:00:00.000Z
// Second: 2024-10-21T01:00:01.000Z
// ※Different times are output because subscription times differ
```

## Difference Between iif and defer

| Feature | iif | defer |
|------|-----|-------|
| **Choice** | Select from two Observables | Generate any Observable |
| **Evaluation Timing** | Evaluate condition at subscription time | Execute function at subscription time |
| **Purpose** | Conditional branching | Dynamic generation |

## Using in Pipeline

Conditional Creation Functions can be used in combination with other operators.

```typescript
import { defer, of } from 'rxjs';
import { switchMap } from 'rxjs';

// Get user information from user ID
const userId$ = of(123);

userId$.pipe(
  switchMap(id =>
    defer(() => {
      // Check latest cache at subscription time
      const cached = cache.get(id);
      return cached ? of(cached) : fetchUser(id);
    })
  )
).subscribe(console.log);
```

## Converting Cold to Hot

As shown in the table above, **all Conditional Creation Functions generate Cold Observables**. Conditional evaluations and generation functions are executed each time a subscription is made.

You can convert a Cold Observable to a Hot Observable by using multicast operators (`share()`, `shareReplay()`, etc.).

### Practical Example: Sharing Conditional Branching Results

```typescript
import { iif, of, interval } from 'rxjs';
import { take, share } from 'rxjs';

const condition = () => Math.random() > 0.5;

// ❄️ Cold - Re-evaluate condition for each subscription
const coldIif$ = iif(
  condition,
  of('Condition is true'),
  interval(1000).pipe(take(3))
);

coldIif$.subscribe(val => console.log('Subscriber 1:', val));
coldIif$.subscribe(val => console.log('Subscriber 2:', val));
// → Each subscriber independently evaluates condition (possibility of different results)

// 🔥 Hot - Share condition evaluation results among subscribers
const hotIif$ = iif(
  condition,
  of('Condition is true'),
  interval(1000).pipe(take(3))
).pipe(share());

hotIif$.subscribe(val => console.log('Subscriber 1:', val));
hotIif$.subscribe(val => console.log('Subscriber 2:', val));
// → Condition evaluated only once, results shared
```

> [!TIP]
> For more information, see [Basic Creation - Converting Cold to Hot](/en/guide/creation-functions/basic/#converting-cold-to-hot).

## Next Steps

To learn the detailed behavior and practical examples of each Creation Function, click on the links from the table above.

Also, by learning [Combination Creation Functions](/en/guide/creation-functions/combination/) and [Selection/Partition Creation Functions](/en/guide/creation-functions/selection/), you can understand the whole picture of Creation Functions.
