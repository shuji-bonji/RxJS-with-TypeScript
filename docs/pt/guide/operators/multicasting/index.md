---
description: Explain RxJS multicasting-related operators, including practical multicast strategies such as using share, shareReplay, publish, and multicast, cold to hot conversion, efficient value delivery to multiple subscribers, and memory leak prevention. Learn implementation patterns for performance optimization with TypeScript type inference for type-safe stream sharing.
---

# Operators Used in Multicasting

RxJS provides several dedicated operators to achieve "multicasting", sharing the same Observable output to multiple subscribers.

This page briefly introduces the typical operators related to multicasting from the **as operators** perspective,
and organizes their usage and points to keep in mind.

> ❗ For conceptual explanations of multicasting, structural explanations using Subjects, and concrete code examples,
> see [Multicasting Mechanism](/pt/guide/subjects/multicasting).

## Main Multicasting-Related Operators

| Operator | Features | Notes |
|--------------|------|------|
| **[share()](/pt/guide/operators/multicasting/share)** | The easiest multicast method. Internally equivalent to `publish().refCount()` | Sufficient for many use cases |
| **[shareReplay()](/pt/guide/operators/multicasting/shareReplay)** | In addition to multicasting, provides recent values when resubscribing | When state reuse is required |
| `publish()` + `refCount()` | Multicast configuration with controllable execution timing | Classic and flexible configuration |
| `multicast()` | Low-level API that explicitly passes `Subject` | Useful when you want to use a custom Subject |

## Comparison of Multicasting Patterns

| Operator | Features | Use Case |
|------------|------|-------------|
| **[share()](/pt/guide/operators/multicasting/share)** | Basic multicast | Simultaneous use across multiple components |
| **[shareReplay(n)](/pt/guide/operators/multicasting/shareReplay)** | Buffer past n values | Late subscription/state sharing |
| `publish() + refCount()` | More fine-grained control possible | When advanced control is needed |
| `multicast(() => new Subject())` | Full customization | When special Subject types are needed |

## Cautions When Using Multicasting

1. **Understanding timing**: Understand that the value you receive depends on when the subscription starts
2. **Lifecycle management**: Especially when using `refCount`, the stream is completed when the number of subscribers reaches zero
3. **Error handling**: If an error occurs in a multicast Observable, it will affect all subscribers
4. **Memory management**: Be aware of memory leaks when using `shareReplay`, etc.
