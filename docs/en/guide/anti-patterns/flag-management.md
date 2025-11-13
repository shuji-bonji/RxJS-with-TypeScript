---
title: Rampant State Management Flags Problem
description: "How to refactor 17 boolean flags in RxJS projects to reactive design: Learn to transform imperative state management into declarative Observable streams"
# outline: deep
---

# Rampant State Management Flags

Even in projects where RxJS has been implemented, it is common to see problems with large numbers of boolean flags running rampant in components. This article describes the causes and remedies based on an actual case where as many as 17 flags exist.

## Real-life examples of the problem

First, let's look at some code we have encountered in the field. The following is a typical example of a mess of state management flags:

```typescript
class ProblematicComponent {
  // 17 flags exist
  isLoading = false;
  isSaving = false;
  isDeleting = false;
  isEditing = false;
  hasError = false;
  isFormDirty = false;
  isFormValid = false;
  isDataLoaded = false;
  isUserAuthenticated = false;
  isModalOpen = false;
  isProcessing = false;
  isInitialized = false;
  isUpdating = false;
  isRefreshing = false;
  hasUnsavedChanges = false;
  isSubmitting = false;
  isValidating = false;

  save() {
    // Complex branching within subscribe
    this.apiService.save(this.data).subscribe({
      next: (result) => {
        if (this.isLoading && !this.isSaving) {
          if (this.isFormValid && this.isDataLoaded) {
            if (!this.hasError && !this.isProcessing) {
              // Actual processing
              this.isSaving = false;
              this.hasUnsavedChanges = false;
            }
          }
        }
      },
      error: (err) => {
        this.isSaving = false;
        this.hasError = true;
        this.isProcessing = false;
      }
    });
  }
}
```

Such code **occurs even with RxJS in place**. This pattern of manually managing 17 flags and controlling them with complex conditional branches is problematic in terms of maintainability, readability, and ease of testing.

## Why the flags are all over the place?

The reasons behind flagging are not only technical issues, but also related to the thinking patterns of developers and the evolutionary process of the organization. Below we analyze the five main causes.

### Structural Analysis of Causes

| Cause Category | Specific Symptoms | Background |
|------------|------------|------|
| **① Imperative thinking remains** | More than 10 flags such as `isLoading`, `isSaving`, `isError`<br>Large number of guards such as `if (this.isSaving) return;` | Logic branches with **imperative "state flag" control** instead of RxJS streams.<br>State and side effects cannot be separated, reducing readability |
| **② Underutilization of derived state** | Managed by directly assigning `this.isLoaded = true;` on the component side | Observable's `map` and `combineLatest` can be used to declaratively define state derivation,<br>but instead manually composing state |
| **③ Ambiguous state design responsibilities** | Multiple flags for the same state<br>(e.g., `isLoadingStart`, `isLoadingEnd`) exist | **Treating state changes as commands**.<br>Distributing what should be integrated as "one state" into multiple flags |
| **④ Unorganized RxJS stream branching** | Multiple `if` and `tap` chains in a single `Observable`,<br>side effects and state updates mixed | Stream design responsibilities are not separated.<br>Usage of `switchMap` and `catchError` is ambiguous |
| **⑤ Lack of ViewModel layer** | Directly manipulate `this.isEditing`, `this.isSaved` in UI components | By holding state in components,<br>the benefits of RxJS are cut off |


## Root cause: mismatch in thinking models

The root cause of the flag confusion is the **mismatch between imperative and reactive programming thought models**. If developers use RxJS with an imperative mindset, the following problems will occur:

### Transitional structure

Many projects go through the following evolutionary process of flagging hell:

```
1. Add if flag control to make it work for now
   ↓
2. Introduce RxJS later
   ↓
3. Old logic cannot be streamed and coexists
   ↓
4. Flag hell is complete
```

### Mixed layers of state management

State within an application should originally be managed in three layers:

```
Application
 ├── View state (isOpen, isLoading, formDirty)     ← Within component
 ├── Business state (entity, filters, errors)      ← State management layer
 └── API state (pending, success, error)           ← RxJS stream
```

If these three layers are not separated, the same "flag" can have **three different responsibilities** mixed together. Managing View state and API state at the same level can lead to an explosion of complexity.

## Nature of the Problem: "Nature" of Flag

The real problem with the flag mess is not that there are "too many of them", but that **the flags are mutable variables of the imperative type**. Below we compare the difference between problematic flags and proper flags.

### ❌ Problematic flags: imperative mutable variables

```typescript
class BadComponent {
  // These are not "states" but "commands"
  isLoading = false;
  isSaving = false;
  hasError = false;

  save() {
    if (this.isSaving) return;        // Guard clause required
    this.isSaving = true;              // Manual change

    this.api.save().subscribe({
      next: () => {
        this.isSaving = false;         // Manual reset
        this.hasError = false;         // Manual management of other flags
      },
      error: () => {
        this.isSaving = false;         // Same process in multiple locations
        this.hasError = true;
      }
    });
  }
}
```

> [!WARNING] Problems
> - State is "procedural" rather than "declarative"
> - Timing of state changes is scattered
> - Developers must manually ensure consistency between flags

### ✅ Appropriate flags: Reactive variables

```typescript
class GoodComponent {
  // Declared as a state stream
  private saveAction$ = new Subject<void>();

  readonly saveState$ = this.saveAction$.pipe(
    switchMap(() =>
      this.api.save().pipe(
        map(() => 'success' as const),
        catchError(() => of('error' as const)),
        startWith('loading' as const)
      )
    ),
    startWith('idle' as const),
    shareReplay(1)
  );

  // Derived states are also defined declaratively
  readonly isLoading$ = this.saveState$.pipe(
    map(state => state === 'loading')
  );

  readonly hasError$ = this.saveState$.pipe(
    map(state => state === 'error')
  );

  save() {
    this.saveAction$.next(); // Event firing only
  }
}
```

> [!TIP] Improvements
> - State is centrally managed as a "stream"
> - State transitions are declaratively defined in pipeline
> - Consistency between flags is automatically guaranteed


## Criteria for flag design

The following is a list of criteria to determine if your code has a problematic flag design. Use them as a reference during code review and design.

| Perspective | ❌ Problematic | ✅ No problem |
|------|-----------|-----------|
| **Type** | `boolean` (mutable) | `Observable<boolean>` / `Signal<boolean>` |
| **Change method** | Direct assignment `flag = true` | Stream/derivation `state$.pipe(map(...))` |
| **Dependencies** | Implicit (code order) | Explicit (combineLatest, computed) |
| **Naming** | `xxxFlag`, `isXXX` (boolean) | `xxxState`, `canXXX`, `shouldXXX` |
| **Count** | More than 10 independent booleans | 1 state + multiple derivations |


## Improvement strategy

To solve the flag disruption problem, refactoring can be done step by step in the following three steps.

### Step 1: State Inventory

First, enumerate all current flags and categorize them by responsibility. This will give you an idea of which flags can be integrated.

```typescript
// Enumerate existing flags and classify responsibilities
interface StateInventory {
  view: string[];      // UI display control (isModalOpen, isEditing)
  business: string[];  // Business logic (isFormValid, hasUnsavedChanges)
  api: string[];       // Communication state (isLoading, isSaving, hasError)
}
```

### Step 2: Enumerate the state

Next, multiple related boolean flags are merged into a single state. For example, `isLoading`, `isSaving`, and `hasError` can all be merged as "request state".

```typescript
// Consolidate multiple booleans into one state
enum RequestState {
  Idle = 'idle',
  Loading = 'loading',
  Success = 'success',
  Error = 'error'
}

// Usage example
class Component {
  saveState: RequestState = RequestState.Idle;
  // isLoading, isSaving, hasError become unnecessary
}
```

### Step 3: Reactivation

Finally, manage the state with Observable or Signal and define the derived state declaratively. This automatically guarantees the integrity of the state.

```typescript
// Managed by Observable or Signal
class ReactiveComponent {
  private readonly apiState$ = new BehaviorSubject<ApiState>({
    loading: false,
    saving: false,
    error: null
  });

  private readonly formState$ = this.form.valueChanges.pipe(
    map(() => ({
      dirty: this.form.dirty,
      valid: this.form.valid
    })),
    startWith({ dirty: false, valid: false })
  );

  // Integrated as a ViewModel
  readonly vm$ = combineLatest([
    this.apiState$,
    this.formState$
  ]).pipe(
    map(([api, form]) => ({
      canSave: !api.saving && form.valid,
      showSpinner: api.loading || api.saving,
      showError: api.error !== null
    }))
  );
}
```


## Example implementation: Refactoring of 17 flags

This section shows the actual process of refactoring the component with 17 flags introduced in the introduction into a reactive design. By comparing Before/After, you will be able to experience the effect of the improvement.

### Before: imperative flag management

First, let's review the problematic code, which has 17 boolean flags in a mess and is controlled by a complex conditional branch.

```typescript
class LegacyComponent {
  isLoading = false;
  isSaving = false;
  isDeleting = false;
  isEditing = false;
  hasError = false;
  isFormDirty = false;
  isFormValid = false;
  isDataLoaded = false;
  isUserAuthenticated = false;
  isModalOpen = false;
  isProcessing = false;
  isInitialized = false;
  isUpdating = false;
  isRefreshing = false;
  hasUnsavedChanges = false;
  isSubmitting = false;
  isValidating = false;

  save() {
    if (!this.isLoading &&
        !this.isSaving &&
        this.isFormValid &&
        !this.hasError &&
        this.isDataLoaded) {
      this.isSaving = true;
      this.apiService.save().subscribe({
        next: () => {
          this.isSaving = false;
          this.hasUnsavedChanges = false;
        },
        error: () => {
          this.isSaving = false;
          this.hasError = true;
        }
      });
    }
  }
}
```

### After: Reactive state management

Next, let's look at the improved code, where the 17 flags are organized into three basic states (apiState$, formState$, dataState$) and one derived state (vm$).

```typescript
import { BehaviorSubject, combineLatest, EMPTY } from 'rxjs';
import { map, switchMap, catchError, startWith } from 'rxjs';

interface ApiState {
  loading: boolean;
  saving: boolean;
  deleting: boolean;
  error: string | null;
}

interface DataState {
  loaded: boolean;
  editing: boolean;
}

class RefactoredComponent {
  // Basic state managed by Observable
  private readonly apiState$ = new BehaviorSubject<ApiState>({
    loading: false,
    saving: false,
    deleting: false,
    error: null
  });

  private readonly formState$ = this.form.valueChanges.pipe(
    map(() => ({
      dirty: this.form.dirty,
      valid: this.form.valid
    })),
    startWith({ dirty: false, valid: false })
  );

  private readonly dataState$ = new BehaviorSubject<DataState>({
    loaded: false,
    editing: false
  });

  // Integrated as a ViewModel (derived state)
  readonly vm$ = combineLatest([
    this.apiState$,
    this.formState$,
    this.dataState$,
    this.authService.isAuthenticated$
  ]).pipe(
    map(([api, form, data, auth]) => ({
      // Derived state for UI display
      canSave: !api.saving && form.valid && data.loaded && auth,
      showSpinner: api.loading || api.saving || api.deleting,
      showError: api.error !== null,
      errorMessage: api.error,
      // Individual states are also disclosed as needed
      isEditing: data.editing,
      formDirty: form.dirty
    }))
  );

  save() {
    // Status checks are automatically performed by the ViewModel
    this.apiState$.next({
      ...this.apiState$.value,
      saving: true,
      error: null
    });

    this.apiService.save().pipe(
      catchError(error => {
        this.apiState$.next({
          ...this.apiState$.value,
          saving: false,
          error: error.message
        });
        return EMPTY;
      })
    ).subscribe(() => {
      this.apiState$.next({
        ...this.apiState$.value,
        saving: false
      });
    });
  }
}
```

### Use on the UI side

Reactive state management also greatly simplifies use on the UI side. There is no need to check multiple flags individually, and only the necessary information needs to be retrieved from the ViewModel.

```typescript
// Before: Direct reference to multiple flags
const isButtonDisabled =
  this.isLoading ||
  this.isSaving ||
  !this.isFormValid ||
  this.hasError ||
  !this.isDataLoaded;

// After: Get derived state from ViewModel
this.vm$.subscribe(vm => {
  const isButtonDisabled = !vm.canSave;
  const showSpinner = vm.showSpinner;
  const errorMessage = vm.errorMessage;
});
```


## Importance of naming conventions

Naming is very important in flag design. Proper naming allows the responsibilities, properties, and life cycle of the flag to be understood at a glance. Conversely, ambiguous naming is a source of confusion.

### ❌ Examples of bad naming

The following types of naming are unclear in intent and reduce maintainability:

```typescript
// What flag? What triggers the change?
userFlag: boolean;
dataFlag: boolean;
checkFlag: boolean;

// Is it a state? Is it an action?
isProcess: boolean;  // Processing? Already processed?
```

### ✅ Good naming example

Proper naming clearly expresses the intent and nature of the state, using Observable (`$` suffix) and Signal to clarify the type of state (State, can, should).

```typescript
// Clear expression of state
readonly userLoadState$: Observable<'idle' | 'loading' | 'loaded' | 'error'>;

// The derived state is also clear in its intent
readonly canSubmit$: Observable<boolean>;
readonly shouldShowSpinner$: Observable<boolean>;

// Example using Signal (available in Angular, Preact, Solid.js, etc.)
readonly userLoadState = signal<LoadState>('idle');
readonly canSubmit = computed(() =>
  this.userLoadState() === 'loaded' && this.formValid()
);
```


## Diagnostic Checklist

Use the following checklist to see if your code is suffering from the flag-running problem. Use it as a reference during code review and design.

```markdown
## 🚨 Danger Signals

- [ ] More than 5 boolean variables
- [ ] More than 3 nested `if` statements in `subscribe`
- [ ] Setting the same flag in multiple places
- [ ] More than 3 names of `isXXXing`
- [ ] Have state in components even though there is a state management layer
- [ ] Multiple names of `xxxFlag`
- [ ] Error handling is scattered across each `subscribe`

## ✅ Signs of Improvement

- [ ] State is managed by `Observable` or `Signal`
- [ ] Derived states are defined with `map`/`computed`
- [ ] State transitions are described declaratively
- [ ] ViewModel pattern is applied
- [ ] Naming clearly expresses intent
```

## Summary

In this article, we have described the causes of the flagging disorder problem in RxJS projects and how to remedy it. Finally, let's look back at some important points.

### The Nature of the Problem

1. **That there are 17 flags** ← This is a symptom
2. **They are imperative mutable variables** ← This is the essence
3. **State transitions are not declarative** ← This is the cause
4. **Ambiguous naming (xxxFlag)** ← This is the source of confusion

### Direction for improvement

To solve the flag confusion problem, the following four changes are necessary:

- **boolean variables** → **Observable/Signal**
- **Direct assignment** → **Stream pipeline**
- **17 independent flags** → **1 state + derived states**
- **xxxFlag** → **xxxState$ / canXXX$**

### Most important

> [!IMPORTANT] Important Principle
> "State is the result of events, not directly controlled by flags"

The introduction of RxJS is a shift in "thought", not "syntax". If we continue to drag out imperative thinking, we will never get rid of flag hell. By viewing state as a stream and designing declaratively, maintainability, readability, and testability are all improved.


## Related Sections

To further your knowledge of flag management learned in this article, please also refer to the following related articles:

- [if statement nesting hell in subscribe](./subscribe-if-hell) - Proper handling of conditional branching
- [Common Mistakes and How to Deal with Them](./common-mistakes) - Details of 15 anti-patterns
- [Error Handling](/en/guide/error-handling/strategies) - Appropriate error handling strategies
- [Subject and Multicasting](/en/guide/subjects/what-is-subject) - Basics of state management

## Reference Resources

Learn more in depth with official RxJS documentation and learning resources:

- [RxJS Official Documentation](https://rxjs.dev/) - Official API reference and guides
- [Learn RxJS](https://www.learnrxjs.io/) - Practical examples by operator
- [RxJS Marbles](https://rxmarbles.com/) - Visual understanding of operator behavior
