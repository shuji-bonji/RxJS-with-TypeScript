---
Description: A learning guide for systematically studying RxJS in a TypeScript environment. It provides step-by-step, practical explanations covering everything from Observable fundamentals to Subjects, various operators, error handling, schedulers, and testing techniques. Each section can be referenced independently.
---

# Guide

This guide helps you systematically learn RxJS in a TypeScript environment.  
By progressing through the sections below in order, you can gain a structured understanding of RxJS from fundamentals to advanced concepts.

## Table of Contents

### 1. RxJS Introduction
- [Getting Started](/en/guide/introduction)
- [Setting Up Your Learning Environment](/en/guide/starter-kid.md)
- [What is RxJS?](/en/guide/basics/what-is-rxjs)
- [What is a Stream?](/en/guide/basics/what-is-a-stream)
- [Promise vs. RxJS](/en/guide/basics/promise-vs-rxjs)

### 2. Observable Fundamentals
- [What is an Observable?](/en/guide/observables/what-is-observable)
- [How to Create an Observable](/en/guide/observables/creation)
- [Streaming Events](/en/guide/observables/events)
- [Events Not Usable with fromEvent](/en/guide/observables/events#cannot-used-fromEvent)
- [Event List](/en/guide/observables/events-list)
- [Observable Lifecycle](/en/guide/observables/observable-lifecycle)
- [Cold Observables and Hot Observables](/en/guide/observables/cold-and-hot-observables)

### 3. Creation Functions
- [What are Creation Functions?](/en/guide/creation-functions/)
- [Basic Creation Functions](/en/guide/creation-functions/basic/) - of, from, fromEvent, interval, timer
- [Loop Generation Functions](/en/guide/creation-functions/loop/) - range, generate
- [HTTP Communication Functions](/en/guide/creation-functions/http-communication/) - ajax, fromFetch
- [Combination Functions](/en/guide/creation-functions/combination/) - concat, merge, combineLatest, zip, forkJoin
- [Selection & Partition Functions](/en/guide/creation-functions/selection/) - race, partition
- [Conditional Functions](/en/guide/creation-functions/conditional/) - iif, defer
- [Control Functions](/en/guide/creation-functions/control/) - scheduled, using

### 4. Understanding Operators
- [Overview of Operators](/en/guide/operators/)
- [Pipeline Concepts](/en/guide/operators/pipeline)
- [Transformation Operators](/en/guide/operators/transformation/) - map, scan, mergeMap, switchMap, buffer-related, window-related, etc.
- [Filtering Operators](/en/guide/operators/filtering/) - filter, take, debounceTime, throttleTime, distinct, etc.
- [Combination Operators](/en/guide/operators/combination/) - concatWith, mergeWith, withLatestFrom, *All operators, etc.
- [Utility Operators](/en/guide/operators/utility/) - tap, delay, retry, finalize, takeUntil, etc.
- [Conditional Operators](/en/guide/operators/conditional/) - defaultIfEmpty, every, isEmpty, etc.
- [Multicasting](/en/guide/operators/multicasting/) - share, shareReplay, etc.

### 5. Subjects and Multicasting
- [What is a Subject?](/en/guide/subjects/what-is-subject)
- [Types of Subject](/en/guide/subjects/types-of-subject)
- [How Multicasting Works](/en/guide/subjects/multicasting)
- [Subject Use Cases](/en/guide/subjects/use-cases)

### 6. Error Handling
- [Error Handling Strategies](/en/guide/error-handling/strategies)
- [Two Locations for Error Handling](/en/guide/error-handling/error-handling-locations)
- [Integrating try-catch with RxJS](/en/guide/error-handling/try-catch-integration)
- [retry and catchError](/en/guide/error-handling/retry-catch)
- [finalize and complete](/en/guide/error-handling/finalize)

### 7. Utilizing Schedulers
- [Controlling Asynchronous Processing](/en/guide/schedulers/async-control)
- [Scheduler Types and Usage](/en/guide/schedulers/types)
- [Supplement: Task and Scheduler Basics](/en/guide/schedulers/task-and-scheduler-basics)

### 8. RxJS Debugging Techniques
- [Debugging Techniques Overview](/en/guide/debugging/)
- [Common Debugging Scenarios](/en/guide/debugging/common-scenarios)
- [Custom Debugging Tools](/en/guide/debugging/custom-tools)
- [Performance Debugging](/en/guide/debugging/performance)

### 9. Testing Techniques
- [RxJS Unit Testing](/en/guide/testing/unit-tests)
- [Using TestScheduler](/en/guide/testing/test-scheduler)
- [Marble Testing](/en/guide/testing/marble-testing)

### 10. RxJS Anti-Patterns Collection
- [What Are Anti-Patterns?](/en/guide/anti-patterns/)
- [Common Mistakes and Solutions](/en/guide/anti-patterns/common-mistakes)
- [Nested if Statements in subscribe](/en/guide/anti-patterns/subscribe-if-hell)
- [Mixing Promises and Observables](/en/guide/anti-patterns/promise-observable-mixing)
- [One-Liner Hell and Separation of Concerns](/en/guide/anti-patterns/one-liner-hell)
- [Anti-Pattern Avoidance Checklist](/en/guide/anti-patterns/checklist)

### 11. Overcoming RxJS Difficulties
- [Why RxJS is Difficult](/en/guide/overcoming-difficulties/)
- [Conceptual Understanding Barrier](/en/guide/overcoming-difficulties/conceptual-understanding)
- [The Lifecycle Management Hurdle](/en/guide/overcoming-difficulties/lifecycle-management)
- [Operator Selection Dilemmas](/en/guide/overcoming-difficulties/operator-selection)
- [Understanding Timing and Order](/en/guide/overcoming-difficulties/timing-and-order)
- [Difficulty with State Management](/en/guide/overcoming-difficulties/state-and-sharing)
- [Combining Multiple Streams](/en/guide/overcoming-difficulties/stream-combination)
- [Debugging Challenges](/en/guide/overcoming-difficulties/debugging-guide)

### 13. Practical Patterns Collection
- [Overview of Practical Patterns](/en/guide/practical-patterns/)
- [UI Event Handling](/en/guide/practical-patterns/ui-events) - Clicks, scrolls, drag & drop, etc.
- [API Calls](/en/guide/practical-patterns/api-calls) - HTTP communication, parallel/serial processing, error handling
- [Form Handling](/en/guide/practical-patterns/form-handling) - Real-time validation, auto-save, multi-field coordination
- [Real-time Data Processing](/en/guide/practical-patterns/real-time-data) - WebSocket, SSE, Polling, connection management
- [Caching Strategies](/en/guide/practical-patterns/caching-strategies) - Data caching, TTL, invalidation, offline support
- [Error Handling Patterns](/en/guide/practical-patterns/error-handling-patterns) - API call errors, retry strategies, global error handling
- [Conditional Branching in Subscriptions](/en/guide/practical-patterns/subscribe-branching) - Avoiding branching within subscriptions, branching methods within pipelines

### Appendix
- [Appendix Overview](/en/guide/appendix/)
- [Embedded Development and Reactive Programming](/en/guide/appendix/embedded-reactive-programming)
- [Reactive Patterns Beyond RxJS](/en/guide/appendix/reactive-patterns-beyond-rxjs)
- [Reactive Architecture Map](/en/guide/appendix/reactive-architecture-map)

---

> [!NOTE]
> This guide is structured to deepen your understanding of RxJS in a step-by-step and systematic manner.
> Feel free to reference any section as needed.
Description: A learning guide for systematically studying RxJS in a TypeScript environment. It provides step-by-step, practical explanations from Observable fundamentals to Subjects, various operators, error handling, schedulers, and testing techniques. Each section can be referenced independently.
