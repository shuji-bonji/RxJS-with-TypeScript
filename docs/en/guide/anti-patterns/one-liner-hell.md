---
description: This article details the stage separation syntax that resolves RxJS "one-liner hell." By clearly separating stream definition, transformation, and subscription, you can achieve easy-to-debug, easy-to-test, and readable code.
---

# One-liner Hell and Stage Separation Syntax

The main reason why RxJS code looks like "one-liner hell" is that **"stream definitions," "transformations," and "subscriptions (side effects)" are jumbled together**. This significantly reduces readability and debuggability.

## Why the "one-liner hell"?

### ❌ Common Problem Codes

```ts
import { fromEvent } from 'rxjs';
import { map, filter, debounceTime, switchMap } from 'rxjs';
import { ajax } from 'rxjs/ajax';

fromEvent(document, 'click')
  .pipe(
    map(ev => (ev as MouseEvent).clientX),
    filter(x => x > 100),
    debounceTime(300),
    switchMap(x => ajax(`/api?x=${x}`))
  )
  .subscribe(res => {
    if (res.status === 200) {
      console.log('OK');
    } else {
      handleError(res);
    }
  });

function handleError(res: any) {
  console.error('Error:', res);
}
```

### Problems

| Problem | Impact |
|---|---|
| **Long lines** | Readers get lost |
| **Difficult to debug** | Hard to check intermediate states |
| **Difficult to test** | Can only test the entire stream |
| **Processing structure is nested** | Conditional branching tends to be deep in subscribe |
| **Not reusable** | Pipeline processing cannot be used elsewhere |


## Solution: Stage Separation Syntax (Functional Style)

Organize RxJS code into a "three-stage structure with clear relationships":

1. **Stream definition (source)** - Source of data
2. **Stream transformation (pipeline)** - Data processing
3. **Subscription and side effects (subscription)** - Side effects such as UI updates and logs


## Recommended pattern: Stage separation syntax

```ts
import { fromEvent } from 'rxjs';
import { map, filter, throttleTime } from 'rxjs';

// 1. Observable definition (stream source)
const clicks$ = fromEvent(document, 'click');

// 2. Pipeline definition (data transformation)
const processed$ = clicks$.pipe(
  map(event => (event as MouseEvent).clientX),
  filter(x => x > 100),
  throttleTime(200)
);

// 3. Subscription processing (side effect execution)
const subscription = processed$.subscribe({
  next: x => console.log('Click position:', x),
  error: err => console.error(err),
  complete: () => console.log('Complete')
});
```

### Advantages

| Advantage | Detail |
|---|---|
| **Clear meaning at each step** | Responsibilities of each stage are clear at a glance |
| **Easy to debug** | Intermediate streams can be checked with `console.log` or `tap` |
| **Easy to test** | Intermediate streams such as `processed$` can be tested independently |
| **Shallow nesting** | Processing in subscribe is simple |
| **Reusable** | Pipeline processing can be extracted as functions |


## Variation: Function separation (modularization)

If the conversion process is long, **separate the pipeline as functions**.

```ts
import { Observable } from 'rxjs';
import { map, filter, distinctUntilChanged } from 'rxjs';
import { fromEvent } from 'rxjs';

// Extract pipeline processing as a function
function transformClicks(source$: Observable<Event>): Observable<number> {
  return source$.pipe(
    map(ev => (ev as MouseEvent).clientX),
    filter(x => x > 100),
    distinctUntilChanged()
  );
}

// Usage side
const clicks$ = fromEvent(document, 'click');
const xPosition$ = transformClicks(clicks$);
const subscription = xPosition$.subscribe(x => console.log(x));
```

**Point:** Extracting "how to convert" as a pure function **explodes testability**.


## Naming Rule

Appropriate naming clarifies the intent of the code.

| Stage | Naming example | Meaning |
|---|---|---|
| **Source** | `clicks$`, `input$`, `routeParams$` | Event or data source |
| **Pipe** | `processed$`, `validInput$`, `apiResponse$` | Processed stream |
| **Subscription** | `subscription`, `uiSubscription` | Actually executed side effects |

**`$` suffix** makes it clear at a glance that the code is Observable.


## For more declarative writing (RxJS 7 or later)

Cut out `pipe` as a function and make it reusable.

```ts
import { pipe, fromEvent } from 'rxjs';
import { map, filter } from 'rxjs';

// Define pipeline as a function (reusable)
const processClicks = pipe(
  map((ev: MouseEvent) => ev.clientX),
  filter(x => x > 100)
);

const clicks$ = fromEvent(document, 'click');
const processed$ = clicks$.pipe(processClicks);
processed$.subscribe(x => console.log(x));
```

**Advantage:** Processing logic (`processClicks`) can be reused in other streams.


## Before/After: Refactor by typical pattern

Examples of improvements in actual use cases are presented here.

### A. UI Event → API → UI Update

#### ❌ Before (one-liner hell)

```ts
import { fromEvent } from 'rxjs';
import { throttleTime, switchMap, catchError } from 'rxjs';
import { ajax } from 'rxjs/ajax';
import { of } from 'rxjs';

interface ApiRes {
  items: string[];
  error?: string;
}

const button = document.getElementById('btn') as HTMLButtonElement;
const list = document.getElementById('list') as HTMLElement;

fromEvent(button, 'click').pipe(
  throttleTime(500),
  switchMap(() => ajax.getJSON<ApiRes>('/api/items')),
  catchError(err => of({ items: [], error: err.message }))
).subscribe(res => {
  list.innerHTML = res.items.map(item => `<li>${item}</li>`).join('');
  if (res.error) alert(res.error);
});
```

#### ✅ After (step separation + functionalization)

```ts
import { fromEvent, pipe, of } from 'rxjs';
import { throttleTime, switchMap, map, catchError } from 'rxjs';
import { ajax } from 'rxjs/ajax';

interface ApiRes {
  items: string[];
}

interface Result {
  items: string[];
  error: string | null;
}

const button = document.getElementById('btn') as HTMLButtonElement;
const list = document.getElementById('list') as HTMLElement;

// 1) source
const clicks$ = fromEvent(button, 'click');

// 2) pipeline (extracted to pure function)
const loadItems = () =>
  pipe(
    throttleTime(500),
    switchMap(() => ajax.getJSON<ApiRes>('/api/items')),
    map((res: ApiRes) => ({ items: res.items, error: null as string | null })),
    catchError(err => of({ items: [] as string[], error: String(err?.message ?? err) }))
  );

const result$ = clicks$.pipe(loadItems());

// 3) subscription (side effects only)
const subscription = result$.subscribe(({ items, error }) => {
  renderList(items);
  if (error) toast(error);
});

function renderList(items: string[]) {
  list.innerHTML = items.map(item => `<li>${item}</li>`).join('');
}

function toast(message: string) {
  alert(message);
}
```

**Improvements:**
- Pipeline processing `loadItems()` is now a pure function
- Consolidate side effects (`renderList`, `toast`) into subscribe side
- Easier testing and debugging


### B. Form Values → Validation → API Save (Auto Save)

#### ❌ Before

```ts
import { fromEvent } from 'rxjs';
import { map, debounceTime, distinctUntilChanged, filter, switchMap } from 'rxjs';
import { ajax } from 'rxjs/ajax';

const input = document.getElementById('input') as HTMLInputElement;

fromEvent(input, 'input')
  .pipe(
    map((e: Event) => (e.target as HTMLInputElement).value),
    debounceTime(400),
    distinctUntilChanged(),
    filter(v => v.length >= 3),
    switchMap(v => ajax.post('/api/save', { v }))
  )
  .subscribe(
    () => console.log('OK'),
    err => alert(err.message)
  );
```

#### ✅ After (responsibility separation + naming)

```ts
import { fromEvent, pipe } from 'rxjs';
import { map, debounceTime, distinctUntilChanged, filter, switchMap } from 'rxjs';
import { ajax } from 'rxjs/ajax';

const input = document.getElementById('input') as HTMLInputElement;

// 1) source
const value$ = fromEvent<Event>(input, 'input').pipe(
  map(e => (e.target as HTMLInputElement).value)
);

// 2) pipeline (validation)
const validate = () =>
  pipe(
    debounceTime(400),
    distinctUntilChanged(),
    filter((v: string) => v.length >= 3)
  );

// 2) pipeline (auto-save)
const autosave = () =>
  pipe(
    switchMap((v: string) => ajax.post('/api/save', { v }))
  );

const save$ = value$.pipe(validate(), autosave());

// 3) subscription
const subscription = save$.subscribe({
  next: () => showSuccess(),
  error: (err) => showError(String(err?.message ?? err))
});

function showSuccess() {
  console.log('Saved');
}

function showError(message: string) {
  alert(message);
}
```

**Improvements:**
- Separation of validation (`validate`) and save (`autosave`)
- Each pipeline can be reused
- Easier testing (validation and save can be tested separately)


### C. Cache + manual refresh

```ts
import { merge, of, Subject } from 'rxjs';
import { switchMap, shareReplay } from 'rxjs';
import { ajax } from 'rxjs/ajax';

interface Item {
  id: number;
  name: string;
}

const refreshBtn = document.getElementById('refresh-btn') as HTMLButtonElement;

// 1) sources
const refresh$ = new Subject<void>();
const initial$ = of(void 0);

// 2) pipeline
const fetchItems$ = merge(initial$, refresh$).pipe(
  switchMap(() => ajax.getJSON<Item[]>('/api/items')),
  shareReplay({ bufferSize: 1, refCount: true }) // Memoization
);

// 3) subscription
const subscription = fetchItems$.subscribe(items => renderList(items));

// Reload from UI
refreshBtn?.addEventListener('click', () => refresh$.next());

function renderList(items: Item[]) {
  console.log('Items:', items);
}
```

**Point:**
- Separate initial autoload (`initial$`) and manual refresh (`refresh$`)
- Cache latest values with `shareReplay`
- Multiple subscribers share the same results


## Advanced: If you want to embed intermediate logs

You can observe each step with `tap()`.

```ts
import { fromEvent } from 'rxjs';
import { map, tap } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

const processed$ = clicks$.pipe(
  tap(() => console.log('Click occurred')),
  map(e => (e as MouseEvent).clientX),
  tap(x => console.log('X coordinate:', x))
);

processed$.subscribe(x => console.log('Final value:', x));
```

**Point:**
- `tap` is an operator dedicated to side effects
- Can check the value of each step during debugging
- Should be removed in production


## Demonstrating Ease of Testing

Stage separation allows **testing pipeline processing in isolation**.

### Example: Testing input validation

```ts
// validate.ts
import { pipe } from 'rxjs';
import { map, debounceTime, distinctUntilChanged, filter } from 'rxjs';

export const validateQuery = () =>
  pipe(
    map((s: string) => s.trim()),
    debounceTime(300),
    distinctUntilChanged(),
    filter((s) => s.length >= 3)
  );
```

```ts
// validate.spec.ts
import { TestScheduler } from 'rxjs/testing';
import { validateQuery } from './validate';

describe('validateQuery', () => {
  it('trims, debounces, distincts, filters length>=3', () => {
    const scheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });

    scheduler.run(({ hot, expectObservable }) => {
      // Input: " a ", "ab", "abc", "abc ", "abcd"
      const input = hot<string>('-a-b-c--d-e----|', {
        a: ' a ',
        b: 'ab',
        c: 'abc',
        d: 'abc ',
        e: 'abcd'
      });

      const output$ = input.pipe(validateQuery());

      // Expected: only 'abc' and 'abcd' pass through
      expectObservable(output$).toBe('--------c-----e-|', {
        c: 'abc',
        e: 'abcd'
      });
    });
  });
});
```

**Advantages:**
- Pipeline processing can be tested **alone**
- DOM/HTTP independent = **Fast & Stable**
- Marble testing controls time axis

See [Testing Methodology](/en/guide/testing/unit-tests) for more details.


## GitHub Copilot Instruction Template

This is a collection of prompts that can be used in actual refactoring.

### 1. Three-stage decomposition

```
Refactor this RxJS code by breaking it into three stages: "source / pipeline / subscription".
Requirements:
- Name Observables with $ suffix
- Extract pipeline as functions that return pipe(...) (e.g., validate(), loadItems())
- Consolidate side effects (UI updates, console, toast) into subscribe
- Insert tap() in appropriate places to observe intermediate states (with comments)
- Use variable and function names that convey the domain
```

### 2. Clarify operator selection

```
Want to prevent multiple API calls from multiple clicks.
Suggest which of the current switchMap/mergeMap/concatMap/exhaustMap should be used,
and replace with the correct operator. Write the rationale in comments.

Guidelines:
- Form save is sequential (concatMap)
- Search suggestions discard old requests (switchMap)
- Button mashing prohibits double execution (exhaustMap)
```

### 3. Auto-save pattern

```
Refactor the following code into an auto-save pattern:
- Input uses debounceTime and distinctUntilChanged
- Save is serialized with concatMap
- Side effects that notify UI of success/failure are moved to subscribe side
- Functionalize transformations for ease of testing
- Cache latest state with shareReplay if possible
```

### 4. Cache + manual refresh

```
Change to "initial auto-load + manual refresh" pattern:
- Introduce refresh$ Subject
- merge(initial$, refresh$) → switchMap(fetch)
- Cache latest value with shareReplay({bufferSize:1, refCount:true})
- Extract fetch pipe as function for reusability
```


## Conclusion: Summary of guidelines for easy-to-read writing

| Item | Recommended |
|---|---|
| ✅ 1 | **Write Observable, pipe, and subscribe separately** |
| ✅ 2 | **Show meaning with variable names** for intermediate streams |
| ✅ 3 | **Functionalize** complex pipes |
| ✅ 4 | **Enable intermediate checks with tap()** |
| ✅ 5 | Make it reusable with `processSomething = pipe(...)` |


## Summary

- **One-Liner Hell** is caused by mixing stream definition, conversion, and subscription
- **Step separation syntax** (Source → Pipeline → Subscription) clarifies responsibilities
- **Functionalizing pipelines** improves ease of testing and reusability
- **Improved readability with proper naming** (`$` suffix, meaningful variable names)

## Related Sections

- **[Common Mistakes and How to Deal with Them](/en/guide/anti-patterns/common-mistakes#13-overcomplication)** - Anti-pattern of overcomplication
- **[Testing Techniques](/en/guide/testing/unit-tests)** - How to test RxJS code
- **[Understanding Operators](/en/guide/operators/)** - How to use each operator

## Next Steps

1. Look for "one-liner hell" in existing code
2. Refactor with a step separation syntax
3. Write unit tests with pipeline processing as a function
4. Use the Copilot instruction template to make it consistent across the team


> [!NOTE]
> A more comprehensive "How to write readable RxJS" will be covered in a future **Chapter 12: Practical Patterns**.
