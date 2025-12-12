---
description: "Anti-pattern avoidance checklist for RxJS code: 16 essential best practices covering memory leaks, subscriptions, error handling, and operator selection"
---

# Anti-pattern Avoidance Checklist

Use this checklist to ensure your RxJS code does not fall into any anti-patterns. Click on each item to see detailed explanations and code examples.

## Checklist Items

### 🔴 Avoid Critical Problems

| Check | Item | Key Points |
|:---:|---|---|
| <input type="checkbox" /> | **[Publish Subject with asObservable()](./common-mistakes#1-external-publication-of-subject)** | Do not export `Subject` directly, publish it as an Observable with `asObservable()`<br>Allow state changes only through dedicated methods |
| <input type="checkbox" /> | **[Avoid nested subscribe](./common-mistakes#2-nested-subscribe-callback-hell)** | Do not call another `subscribe` within `subscribe`<br>Flatten with `switchMap`, `mergeMap`, `concatMap`, etc. |
| <input type="checkbox" /> | **[Always unsubscribe from infinite streams](./common-mistakes#3-unsubscribe-forgetting-memory-leak)** | Always unsubscribe from infinite streams like event listeners<br>`takeUntil` pattern or `Subscription` management |
| <input type="checkbox" /> | **[Explicitly configure shareReplay](./common-mistakes#4-misuse-of-sharereplay)** | Use the form `shareReplay({ bufferSize: 1, refCount: true })`<br>Enable reference counting to prevent memory leaks |
| <input type="checkbox" /> | **[Avoid nested if statements within subscribe](./subscribe-if-hell)** | Avoid complex conditional branching (3 or more nested levels) within `subscribe`<br>Write declaratively with operators like `filter`, `iif`, `partition` |

### 🟡 Avoid Problems Requiring Attention

| Check | Item | Key Points |
|:---:|---|---|
| <input type="checkbox" /> | **[map is pure function, side effects in tap](./common-mistakes#5-side-effects-in-map)** | Do not change state or output logs within `map`<br>Explicitly separate side effects with `tap` operator |
| <input type="checkbox" /> | **[Use Cold/Hot appropriately](./common-mistakes#6-ignoring-cold-hot-observable-differences)** | Convert HTTP requests to Hot with `shareReplay`<br>Determine whether execution should happen per subscription or be shared |
| <input type="checkbox" /> | **[Convert Promise with from](./promise-observable-mixing)** | Do not mix Promise and Observable<br>Convert to Observable with `from()` for unified processing |
| <input type="checkbox" /> | **[Control high-frequency events](./common-mistakes#8-ignoring-backpressure)** | Control search input with `debounceTime`, scrolling with `throttleTime`<br>Exclude duplicates with `distinctUntilChanged` |

### 🔵 Improve Code Quality

| Check | Item | Key Points |
|:---:|---|---|
| <input type="checkbox" /> | **[Handle errors appropriately](./common-mistakes#9-error-suppression)** | Catch errors with `catchError` and handle appropriately<br>Display user-friendly error messages<br>Retry with `retry` / `retryWhen` as needed |
| <input type="checkbox" /> | **[Properly release DOM events](./common-mistakes#10-dom-event-subscription-leaks)** | Always unsubscribe from `fromEvent` subscriptions<br>Automatically unsubscribe with `takeUntil` when component is destroyed |
| <input type="checkbox" /> | **[Ensure type safety](./common-mistakes#11-lack-of-type-safety-excessive-use-of-any)** | Define interfaces and type aliases<br>Explicitly specify `Observable<T>` type parameters<br>Leverage TypeScript type inference |
| <input type="checkbox" /> | **[Choose appropriate operators](./common-mistakes#12-improper-operator-selection)** | Search: `switchMap`, parallel: `mergeMap`<br>Sequential: `concatMap`, prevent double-click: `exhaustMap` |
| <input type="checkbox" /> | **[Simple processing doesn't need RxJS](./common-mistakes#13-overcomplication)** | Regular JavaScript is sufficient for array processing, etc.<br>Use RxJS for asynchronous processing and event streams |
| <input type="checkbox" /> | **[Manage state reactively](./common-mistakes#14-state-changes-in-subscribe)** | Manage state with `BehaviorSubject` or `scan`<br>Use `subscribe` as final trigger |
| <input type="checkbox" /> | **[Write tests](./common-mistakes#15-lack-of-testing)** | Implement marble testing with `TestScheduler`<br>Make asynchronous processing testable synchronously |

## How to Use

### 1. During Code Review

After writing new code, conduct a self-review using this checklist.

### 2. During Pull Requests

Include this checklist in your pull request template so reviewers can verify with common criteria.

### 3. Regular Reviews

Use this checklist regularly against your existing codebase to check for anti-patterns.

### 4. Share Within Team

Share with team members to unify RxJS best practices.

## Related Resources

- **[Common Mistakes and How to Fix Them](./common-mistakes)** - Detailed explanations and code examples for each anti-pattern
- **[Anti-Patterns Collection Top](./index)** - List of anti-patterns and how to learn
- **[Error Handling](/pt/guide/error-handling/strategies)** - Error handling best practices
- **[Testing Techniques](/pt/guide/testing/unit-tests)** - How to test RxJS code

## Tips for Using the Checklist

1. **Don't try to perfect all items at once**
   - First, prioritize critical problems (🔴)
   - Improve step by step

2. **Set priorities within the team**
   - Adjust importance according to project characteristics
   - Create customized checklists

3. **Consider automation**
   - Automate checks with static analysis tools like ESLint
   - Integrate into CI/CD pipeline

4. **Regular updates**
   - Update according to RxJS version upgrades
   - Reflect insights from team experience

---

**Important**: This checklist is not for writing perfect code, but a guide to avoid common problems. Use it flexibly according to your project context.
