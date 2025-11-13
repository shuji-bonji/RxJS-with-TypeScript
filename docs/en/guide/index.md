---
Description: A learning guide for systematically studying RxJS in a TypeScript environment. It provides step-by-step, practical explanations covering everything from Observable fundamentals to Subjects, various operators, error handling, schedulers, and testing techniques. Each section can be referenced independently.
---

# Guide

This guide helps you systematically learn RxJS in a TypeScript environment.  
By progressing through the sections below in order, you can gain a structured understanding of RxJS from fundamentals to advanced concepts.

## Table of Contents

### 1. RxJS Introduction
- [Getting Started](/guide/introduction)
- [Setting Up Your Learning Environment](/guide/starter-kid.md)
- [What is RxJS?](/guide/basics/what-is-rxjs)
- [What is a Stream?](/guide/basics/what-is-a-stream)
- [Promise vs. RxJS](/guide/basics/promise-vs-rxjs)

### 2. Observable Fundamentals
- [What is an Observable?](/guide/observables/what-is-observable)
- [How to Create an Observable](/guide/observables/creation)
- [Streaming Events](/guide/observables/events)
- [Events Not Usable with fromEvent](/guide/observables/events#cannot-used-fromEvent)
- [Event List](/guide/observables/events-list)
- [Observable Lifecycle](/guide/observables/observable-lifecycle)
- [Cold Observables and Hot Observables](/guide/observables/cold-and-hot-observables)

### 3. Creation Functions
- [What are Creation Functions?](/guide/creation-functions/)
- [Basic Creation Functions](/guide/creation-functions/basic/) - of, from, fromEvent, interval, timer
- [Loop Generation Functions](/guide/creation-functions/loop/) - range, generate
- [HTTP Communication Functions](/guide/creation-functions/http-communication/) - ajax, fromFetch
- [Combination Functions](/guide/creation-functions/combination/) - concat, merge, combineLatest, zip, forkJoin
- [Selection & Partition Functions](/guide/creation-functions/selection/) - race, partition
- [Conditional Functions](/guide/creation-functions/conditional/) - iif, defer
- [Control Functions](/guide/creation-functions/control/) - scheduled, using

### 4. Understanding Operators
- [Overview of Operators](/guide/operators/)
- [Pipeline Concepts](/guide/operators/pipeline)
- [Transformation Operators](/guide/operators/transformation/) - map, scan, mergeMap, switchMap, buffer-related, window-related, etc.
- [Filtering Operators](/guide/operators/filtering/) - filter, take, debounceTime, throttleTime, distinct, etc.
- [Combination Operators](/guide/operators/combination/) - concatWith, mergeWith, withLatestFrom, *All operators, etc.
- [Utility Operators](/guide/operators/utility/) - tap, delay, retry, finalize, takeUntil, etc.
- [Conditional Operators](/guide/operators/conditional/) - defaultIfEmpty, every, isEmpty, etc.
- [Multicasting](/guide/operators/multicasting/) - share, shareReplay, etc.

### 5. Subjects and Multicasting
- [What is a Subject?](/guide/subjects/what-is-subject)
- [Types of Subject](/guide/subjects/types-of-subject)
- [How Multicasting Works](/guide/subjects/multicasting)
- [Subject Use Cases](/guide/subjects/use-cases)

### 6. Error Handling
- [Error Handling Strategies](/guide/error-handling/strategies)
- [Two Locations for Error Handling](/guide/error-handling/error-handling-locations)
- [Integrating try-catch with RxJS](/guide/error-handling/try-catch-integration)
- [retry and catchError](/guide/error-handling/retry-catch)
- [finalize and complete](/guide/error-handling/finalize)

### 7. Utilizing Schedulers
- [Controlling Asynchronous Processing](/guide/schedulers/async-control)
- [Scheduler Types and Usage](/guide/schedulers/types)
- [Supplement: Task and Scheduler Basics](/guide/schedulers/task-and-scheduler-basics)

### 8. RxJS Debugging Techniques
- [Debugging Techniques Overview](/guide/debugging/)
- [Common Debugging Scenarios](/guide/debugging/common-scenarios)
- [Custom Debugging Tools](/guide/debugging/custom-tools)
- [Performance Debugging](/guide/debugging/performance)

### 9. Testing Techniques
- [RxJS Unit Testing](/guide/testing/unit-tests)
- [Using TestScheduler](/guide/testing/test-scheduler)
- [Marble Testing](/guide/testing/marble-testing)

### 10. RxJS Anti-Patterns Collection
- [What Are Anti-Patterns?](/guide/anti-patterns/)
- [Common Mistakes and Solutions](/guide/anti-patterns/common-mistakes)
- [Nested if Statements in subscribe](/guide/anti-patterns/subscribe-if-hell)
- [Mixing Promises and Observables](/guide/anti-patterns/promise-observable-mixing)
- [One-Liner Hell and Separation of Concerns](/guide/anti-patterns/one-liner-hell)
- [Anti-Pattern Avoidance Checklist](/guide/anti-patterns/checklist)

### 11. Overcoming RxJS Difficulties
- [Why RxJS is Difficult](/guide/overcoming-difficulties/)
- [Conceptual Understanding Barrier](/guide/overcoming-difficulties/conceptual-understanding)
- [The Lifecycle Management Hurdle](/guide/overcoming-difficulties/lifecycle-management)
- [Operator Selection Dilemmas](/guide/overcoming-difficulties/operator-selection)
- [Understanding Timing and Order](/guide/overcoming-difficulties/timing-and-order)
- [Difficulty with State Management](/guide/overcoming-difficulties/state-and-sharing)
- [Combining Multiple Streams](/guide/overcoming-difficulties/stream-combination)
- [Debugging Challenges](/guide/overcoming-difficulties/debugging-guide)

### 13. Practical Patterns Collection
- [Overview of Practical Patterns](/guide/practical-patterns/)
- [UI Event Handling](/guide/practical-patterns/ui-events) - Clicks, scrolls, drag & drop, etc.
- [API Calls](/guide/practical-patterns/api-calls) - HTTP communication, parallel/serial processing, error handling
- [Form Handling](/guide/practical-patterns/form-handling) - Real-time validation, auto-save, multi-field coordination
- [Real-time Data Processing](/guide/practical-patterns/real-time-data) - WebSocket, SSE, Polling, connection management
- [Caching Strategies](/guide/practical-patterns/caching-strategies) - Data caching, TTL, invalidation, offline support
- [Error Handling Patterns](/guide/practical-patterns/error-handling-patterns) - API call errors, retry strategies, global error handling
- [Conditional Branching in Subscriptions](/guide/practical-patterns/subscribe-branching) - Avoiding branching within subscriptions, branching methods within pipelines

### Appendix
- [Appendix Overview](/guide/appendix/)
- [Embedded Development and Reactive Programming](/guide/appendix/embedded-reactive-programming)
- [Reactive Patterns Beyond RxJS](/guide/appendix/reactive-patterns-beyond-rxjs)
- [Reactive Architecture Map](/guide/appendix/reactive-architecture-map)

---

> [!NOTE]
> This guide is structured to deepen your understanding of RxJS in a step-by-step and systematic manner.
> Feel free to reference any section as needed.
Description: A learning guide for systematically studying RxJS in a TypeScript environment. It provides step-by-step, practical explanations from Observable fundamentals to Subjects, various operators, error handling, schedulers, and testing techniques. Each section can be referenced independently.
