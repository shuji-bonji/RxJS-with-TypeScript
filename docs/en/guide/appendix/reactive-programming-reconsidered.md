---
description: "Is Reactive Programming really a panacea? We will examine the gap between design philosophy and reality, and objectively explain the strengths and limitations of RP, as well as areas that should be applied and those that should be avoided."
titleTemplate: ':title'
---

# RP Reconsidered - Design vs Reality

Reactive Programming (RP) is widely known as a powerful paradigm for asynchronous data stream processing.

But is **RP really a panacea?** This page examines the gap between RP ideals and reality, and objectively considers where RP should and should not be used.


## RP ideals vs. reality

### Ideal: Sophisticated modern design

RPs are often advertised as follows

- **Declarative** and easy-to-read code
- **Concise** expression of asynchronous processing
- **handles complex data flows** in a unified manner
- **core technology of reactive architecture**.

### Reality: It can reduce team productivity.

However, the following problems have been encountered in actual projects

- **very high learning curve**
- **Difficult to debug**
- **Complex testing**
- **Loss of productivity due to misuse**

> [!WARNING]
> Applying RP to "all code" may conversely increase code complexity and reduce team productivity.

## Four challenges facing RP

### 1. Height of learning curve

Mastering RP requires a different thought model than traditional imperative programming.

#### Difficult to track data flow

```typescript
// ❌ Data flow is difficult to see
source$
  .pipe(
    mergeMap(x => fetchData(x)),
    switchMap(data => processData(data)),
    concatMap(result => saveData(result))
  )
  .subscribe(/*...*/);
```

::: warning problem
- Differences between `mergeMap`, `switchMap`, and `concatMap` are not intuitive
- Difficult to track where and how data is transformed
- Difficult to identify where the error occurred
:::

#### Debugging and logging difficulties

```typescript
// Difficult to debug
source$
  .pipe(
    map(x => x * 2),
    filter(x => x > 10),
    mergeMap(x => api(x))
  )
  .subscribe(/*...*/);

// Where did the error occur?
// Which operator lost the value?
```

> [!TIP]
> The `tap()` operator is used for debugging, which itself is an additional learning cost.
> ```typescript
> source$
>   .pipe(
>     tap(x => console.log('Before map:', x)),
>     map(x => x * 2),
>     tap(x => console.log('After map:', x)),
>     filter(x => x > 10),
>     tap(x => console.log('After filter:', x))
>   )
>   .subscribe(/*...*/);
> ```

### 2. High cognitive load

There are more than 100 operators in the RP, and their use is complex.

#### Too many choices of operators.

| criterion | options | difference |
|------|--------|------|
| Sequential processing of arrays | `concatMap`, `mergeMap`, `switchMap`, `exhaustMap` | Concurrency and order guarantees differ |
| Combining Multiple Streams | `concat`, `merge`, `combineLatest`, `zip`, `forkJoin`, `race` | Different bonding methods |
| error handling | `catchError`, `retry`, `retryWhen`, `onErrorResumeNext` | Retry strategies differ |

**Why bother writing a process in RP that can be done with a simple `if` or `await`?**

```typescript
// ❌ Example of complex writing in RP
of(user)
  .pipe(
    mergeMap(u => u.isPremium
      ? fetchPremiumData(u)
      : fetchBasicData(u)
    )
  )
  .subscribe(/*...*/);

// ✅ Simple conditional branching
const data = user.isPremium
  ? await fetchPremiumData(user)
  : await fetchBasicData(user);
```

### 3. Difficulty of the test

Testing RP requires an understanding of time control and Marble Testing.

#### Marble Testing's Learning Cost

```typescript
import { TestScheduler } from 'rxjs/testing';

it('debounceTime test', () => {
  const testScheduler = new TestScheduler((actual, expected) => {
    expect(actual).toEqual(expected);
  });

  testScheduler.run(({ cold, expectObservable }) => {
    const input$  = cold('-a-b-c---|');
    const expected =     '-----c---|';

    const result$ = input$.pipe(debounceTime(50, testScheduler));

    expectObservable(result$).toBe(expected);
  });
});
```

::: warning problem
- Need to learn Marble Diagram notation
- Need to understand time control mechanism
- Higher learning cost than normal unit testing
:::

#### Frequent synchronization bugs

```typescript
// ❌ Common bugs: subscription timing issues
const subject$ = new Subject();

subject$.next(1);  // this value is not accepted
subject$.subscribe(x => console.log(x));  // subscription is slow
subject$.next(2);  // I can take this.
```

### 4. Complications due to misuse

Applying RP to all codes creates unnecessary complexity.

#### Over-application to simple CRUD processes

```typescript
// ❌ Excessive RP application
getUserById(userId: string): Observable<User> {
  return this.http.get<User>(`/api/users/${userId}`)
    .pipe(
      map(user => this.transformUser(user)),
      catchError(error => {
        console.error('Error:', error);
        return throwError(() => error);
      })
    );
}

// ✅ Simple Promise
async getUserById(userId: string): Promise<User> {
  try {
    const user = await fetch(`/api/users/${userId}`).then(r => r.json());
    return this.transformUser(user);
  } catch (error) {
    console.error('Error:', error);
    throw error;
  }
}
```

> [!IMPORTANT]
> **RP is not a "silver bullet that solves all problems."** It is important to identify areas to apply and areas to avoid.

## Areas where RP excels

RP is not a panacea, but it is very powerful in the following areas

### 1. Continuous data stream processing

Ideal for processing **continuously occurring data** such as sensor data, log streams, real-time data, etc.

```typescript
// ✅ Example of where RP has an advantage: processing sensor data
sensorStream$
  .pipe(
    filter(reading => reading.value > threshold),
    bufferTime(1000),                           // Aggregate every second
    map(readings => calculateAverage(readings)),
    distinctUntilChanged()                      // Notify only when there is a change
  )
  .subscribe(avg => updateDashboard(avg));
```

### 2. WebSockets and push notifications

Ideal for bidirectional communication and push-type data delivery from servers.

```typescript
// ✅ Reactive processing of WebSocket communication
const socket$ = webSocket('wss://example.com/socket');

socket$
  .pipe(
    retry({ count: 3, delay: 1000 }),  // Auto-reconnect
    map(msg => parseMessage(msg)),
    filter(msg => msg.type === 'notification')
  )
  .subscribe(notification => showNotification(notification));
```

### 3. state management system

NgRx, Redux Observable, MobX, etc., are effective as a foundation for state management libraries.

```typescript
// ✅ RP utilization in state management (NgRx Effects)
loadUsers$ = createEffect(() =>
  this.actions$.pipe(
    ofType(UserActions.loadUsers),
    mergeMap(() =>
      this.userService.getUsers().pipe(
        map(users => UserActions.loadUsersSuccess({ users })),
        catchError(error => of(UserActions.loadUsersFailure({ error })))
      )
    )
  )
);
```

### 4. Back-end non-blocking I/O

Suitable for asynchronous backend processing such as Node.js Streams, Spring WebFlux, Vert.x, etc.

```typescript
// ✅ RP-like processing of Node.js Streams
const fileStream = fs.createReadStream('large-file.txt');
const transformStream = new Transform({
  transform(chunk, encoding, callback) {
    const processed = processChunk(chunk);
    callback(null, processed);
  }
});

fileStream.pipe(transformStream).pipe(outputStream);
```

### 5. event driven distributed system

Kafka, RabbitMQ, Akka Streams, etc. are effective as the foundation for event-driven architectures.

## Areas where RP is unsuitable

In the following areas, the code is simpler and more maintainable without RP.

### 1. Simple CRUD processing

For simple read/write operations to the database, `async`/`await` is more suitable.

```typescript
// ❌ No need to write in RP
getUser(id: string): Observable<User> {
  return this.http.get<User>(`/api/users/${id}`);
}

// ✅ async/await is sufficient
async getUser(id: string): Promise<User> {
  return await fetch(`/api/users/${id}`).then(r => r.json());
}
```

### 2. Simple conditional branching

There is no need to stream a process that can be done with a simple `if` statement.

```typescript
// ❌ Excessive RP application
of(value)
  .pipe(
    mergeMap(v => v > 10 ? doA(v) : doB(v))
  )
  .subscribe();

// ✅ Simple conditional branching
if (value > 10) {
  doA(value);
} else {
  doB(value);
}
```

### 3. One-time asynchronous processing

If Promise is sufficient, there is no need to make it Observable.

```typescript
// ❌ Unnecessary Observable
from(fetchData()).subscribe(data => process(data));

// ✅ Promise is good enough
fetchData().then(data => process(data));
```

## Evolution of RP: Toward simpler abstraction

The philosophy of RP is not disappearing, but is **evolving** into a simpler, more transparent form.

### Angular Signals (Angular 19+)

```typescript
// Signal-based reactivity
const count = signal(0);
const doubled = computed(() => count() * 2);

effect(() => {
  console.log('Count:', count());
});

count.set(5);  // Simple and intuitive
```


::: info Features:
- Lower learning cost than RxJS
- Easy to debug
- Fine-grained reactivity
:::

### React Concurrent Features

```typescript
// React 18's Concurrent Rendering
function UserProfile({ userId }) {
  const user = use(fetchUser(userId));  // Integrated with Suspense
  return <div>{user.name}</div>;
}
```

::: info Features:
- Declarative data fetching
- Automatic priority control
- Hides RP complexity
:::

### Svelte 5 Runes

```typescript
// Svelte 5's Runes ($state, $derived)
let count = $state(0);
let doubled = $derived(count * 2);

function increment() {
  count++;  // Intuitive updates
}
```

::: info Features:
- Optimization by compiler
- No boilerplate
- Transparency of reactivity
:::

> [!TIP]
> These new abstractions significantly reduce **complexity** while retaining the RP's **core value (reactivity)**.

## Policy for appropriate use of RP

### 1. Determine problem areas

| Suitable for. | Not suitable for |
|-----------|-------------|
| continuous data stream | Simple CRUD |
| WebSocket Communication | One-time API call |
| Multiple asynchronous event integration | Simple conditional branching |
| real-time data processing | Static data conversion |
| state management | Simple variable update |

### 2. Phased introduction

```typescript
// ❌ Don't introduce all of them out of the blue
class UserService {
  getUser$ = (id: string) => this.http.get<User>(`/api/users/${id}`);
  updateUser$ = (user: User) => this.http.put<User>(`/api/users/${user.id}`, user);
  deleteUser$ = (id: string) => this.http.delete(`/api/users/${id}`);
  // Make everything Observable
}

// ✅ RP only where necessary
class UserService {
  async getUser(id: string): Promise<User> { /* ... */ }
  async updateUser(user: User): Promise<User> { /* ... */ }

  // Observable only where real-time updates are required
  watchUser(id: string): Observable<User> {
    return this.websocket.watch(`/users/${id}`);
  }
}
```

### 3. Consider the team's proficiency level

| Team Status | Recommended Approach |
|------------|---------------|
| Unfamiliar with RP | Limited implementation (only where there are clear benefits such as WebSockets) |
| Partially mastered | Gradual expansion (state management, real-time processing) |
| All are proficient | Full stack utilization (front-end to back-end) |

### 4. Compare with alternatives

```typescript
// Pattern 1: RP (when multiple events need to be integrated)
combineLatest([
  formValue$,
  validation$,
  apiStatus$
]).pipe(
  map(([value, isValid, status]) => ({
    canSubmit: isValid && status === 'ready',
    value
  }))
);

// Pattern 2: Signals (simpler reactivity)
const formValue = signal({});
const validation = signal(false);
const apiStatus = signal('ready');
const canSubmit = computed(() =>
  validation() && apiStatus() === 'ready'
);

// Pattern 3: async/await (one-time processing)
async function submitForm() {
  const isValid = await validateForm(formValue);
  if (!isValid) return;

  const result = await submitToApi(formValue);
  return result;
}
```

## Summary

### RP is not a panacea

> [!IMPORTANT]
> Reactive Programming is neither **harmful nor a panacea**. It is a **specialized tool** optimized for asynchronous and event flow problems.

### Appreciate the value of RP, but understand its limitations.

::: tip Areas where RP excels
- Continuous data stream processing
- WebSocket and real-time communication
- State management systems
- Back-end non-blocking I/O
- Event-driven distributed systems
:::

::: warning Areas where RP is unsuitable
- Simple CRUD processing
- Simple conditional branching
- One-time asynchronous processing
:::

### Moving to a new abstraction

The philosophy of RP is evolving into **simpler and more transparent forms** such as Angular Signals, React Concurrent Features, and Svelte Runes.

### Guidelines for Application in Practice

1.**Identify problem areas** - Is RP really necessary?
2.**Implement in phases** - don't just adopt it out of the blue
3.**Consider team proficiency** - learning costs are high
4.**Compare with alternatives** - are async/await and Signals sufficient?

> [!TIP]
> **"Use the right tools, in the right place."** This is the key to a successful RP.

## Related Pages

- [Reactive Architecture Map](/en/guide/appendix/reactive-architecture-map) - 7 layers where RP is active
- [RxJS and Reactive Streams Ecosystem](/en/guide/appendix/rxjs-and-reactive-streams-ecosystem) - The entire RP technology stack
- [Overcoming RxJS Difficulties](/en/guide/overcoming-difficulties/) - Overcoming RP learning barriers
- [RxJS Anti-Patterns](/en/guide/anti-patterns/) - Avoiding misuse of RP

## References

- [GitHub Discussion #17 - Reactive Programming Reconsidered](https://github.com/shuji-bonji/RxJS-with-TypeScript/discussions/17)
- [Angular Signals Official Documentation](https://angular.dev/guide/signals)
- [React Concurrent Features](https://react.dev/blog/2022/03/29/react-v18)
- [Svelte 5 Runes](https://svelte.dev/docs/svelte/what-are-runes)
