---
description: A practical guide to understanding RxJS anti-patterns and writing more robust and maintainable code. We will explain 16 common mistakes and how to deal with them, including misuse of Subject, nested subscribe, conditional branching in subscribe, memory leaks, and misuse of shareReplay.
---

# RxJS Anti-Pattern Collection

RxJS is a powerful reactive programming library, but when used incorrectly, it can be a breeding ground for bugs and reduced maintainability. In this section, we will introduce some common mistakes when using RxJS with TypeScript and best practices to avoid them.

## Purpose of this section

- **Prevent bugs**: avoid implementation problems by understanding common mistakes before they happen
- **Improve Maintainability**: Learn code patterns that are easy to read and test
- **Optimize performance**: learn techniques to avoid memory leaks and unnecessary processing

## List of anti-patterns

This section covers the following 17 anti-patterns.

### 🔴 Critical Issues

These patterns can have a serious impact on your application.

| Pattern | Problem | Impact |
|---|---|---|
| **[External publication of Subject](./common-mistakes#1-external-publication-of-subject)** | Publish the Subject as it is, and make it possible to call `next()` from the outside | Unpredictability of state management, difficult to debug |
| **[Nested subscribe](./common-mistakes#2-nested-subscribe-callback-hell)** | Calling more `subscribe` in `subscribe` | Callback hell, complication of error handling |
| **[Rampant state management flags](./flag-management)** | Managing state with 17 boolean flags, imperative thinking still remains | Low readability, difficult to maintain, hotbed of bugs |
| **[if statement nesting in subscribe](./subscribe-if-hell)** | Complex conditional branching (nesting of 3 or more) in `subscribe` | Low readability, difficult to test, violates declarative philosophy |
| **[unsubscribe forgetting](./common-mistakes#3-unsubscribe-forgetting-memory-leak)** | Not unsubscribing to infinite streams | Memory leak, resource waste |
| **[Misuse of shareReplay](./common-mistakes#4-misuse-of-sharereplay)** | Use without understanding how `shareReplay` works | Old data references, memory leaks |

### 🟡 Issues requiring attention

These can be problems in certain situations.

| Pattern | Problem | Impact |
|---|---|---|
| **[Side-effects in map](./common-mistakes#5-side-effects-in-map)** | Changing state in `map` operator | Unpredictable behavior, difficult to test |
| **[Ignoring Cold/Hot](./common-mistakes#6-ignoring-cold-hot-observable-differences)** | Not considering the nature of Observable | Duplicate execution, unexpected behavior |
| **[Mixing with Promise](./promise-observable-mixing)** | Not converting Promise and Observable properly | Uncancelable, poor error handling |
| **[Ignoring backpressure](./common-mistakes#8-ignoring-backpressure)** | Fail to control high frequency events | Performance degradation, UI freeze |

### 🔵 Code quality issues

These are not direct bugs, but they are factors that reduce code quality.

| Pattern | Problem | Impact |
|---|---|---|
| **[Error suppression](./common-mistakes#9-error-suppression)** | Not handling errors properly | Debugging difficulties, poor user experience |
| **[DOM event leaks](./common-mistakes#10-dom-event-subscription-leaks)** | Not freeing DOM event listeners | Memory leaks, performance degradation |
| **[Lack of type safety](./common-mistakes#11-lack-of-type-safety-excessive-use-of-any)** | Excessive use of `any` | Runtime errors, refactoring difficulties |
| **[Improper operator selection](./common-mistakes#12-improper-operator-selection)** | Use of operators that do not serve the purpose | Inefficiency, unexpected behavior |
| **[Overcomplication](./common-mistakes#13-overcomplication)** | Complicating a process that can be written simply | Low readability, difficult to maintain |
| **[One-liner hell](./one-liner-hell)** | Mixed stream definitions, conversions, and subscriptions | Difficult to debug, difficult to test, low readability |
| **[State changes in subscribe](./common-mistakes#14-state-changes-in-subscribe)** | Change of state directly in `subscribe` | Difficult to test, cause bugs |
| **[Lack of tests](./common-mistakes#15-lack-of-testing)** | No tests written for RxJS code | Regression, refactoring difficulties |

## How to proceed with the study

1. Learn 15 anti-patterns in detail in **[Common Mistakes and How to Fix Them](./common-mistakes)**
2. Each anti-pattern has a "bad example" and a "good example" code
3. Review your code with **[Anti-pattern Avoidance Checklist](./checklist)**
4. Practice best practices and share them with your team

## Related Sections

After learning about anti-patterns, please also refer to the following sections.

- **[Error Handling](/pt/guide/error-handling/strategies)** - Appropriate error handling strategies
- **[Testing Techniques](/pt/guide/testing/unit-tests)** - How to test RxJS code
- **[Understanding Operators](/pt/guide/operators/)** - How to choose the right operators

## Next Steps

1. Start with **[Common Mistakes and How to Fix Them](./common-mistakes)** to learn practical anti-patterns and their solutions.
2. After learning, review your actual code with the **[Anti-pattern Avoidance Checklist](./checklist)**.

---

**IMPORTANT**: These anti-patterns are frequently found in real projects. Understanding them early on will help you write quality RxJS code.
