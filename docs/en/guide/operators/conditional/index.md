---
description: RxJS conditional operators are operators for making conditional decisions on values in a stream, setting default values, and evaluating conditions. defaultIfEmpty, every, isEmpty, etc. can be used to implement practical scenarios such as processing empty streams, checking all values, and checking for existence with TypeScript's type safety.
---

# Conditional Operators

RxJS's conditional operators are used to **conditionally determine or evaluate** the value of a stream.
Such as setting a default value for an empty stream, or checking if all values satisfy a condition,
they can be utilized in practical scenarios.

This page introduces each operator in three stages: "Basic Syntax and Operation," "Typical Usage Examples," and "Practical Code Examples (with UI)" in the following structure.

Understand what kind of use cases each operator is suitable for,
and combining them will enable you to design reactive processing that is more robust and in line with your intentions.

> [!NOTE]
> `iif` and `defer` are **Creation Functions** (Observable creation functions) and are not conditional operators. See [Chapter 3: Creation Functions](/en/guide/creation-functions/) for these.

## List of Operators

Below is a list of the major conditional operators and their characteristics.

| Operator | Description |
|--------------|------|
| [defaultIfEmpty](./defaultIfEmpty.md) | Alternative value when no value is emitted |
| [every](./every.md) | Evaluate whether all values match a condition |
| [isEmpty](./isEmpty.md) | Check if any value is emitted |

> For **practical combinations** and **use-case based applications** of operators, see the [Practical Use Cases](./practical-use-cases.md) section at the end.


## Be Aware of Integration with Other Categories

Conditional operators are only useful in combination with other transformation, combination, and utility operators.
For example, it is common to combine them with `switchMap` and `catchError` to perform "API switching and recovery processing".

For more practical use cases, see [Practical Use Cases](./practical-use-cases.md) for detailed explanations.
