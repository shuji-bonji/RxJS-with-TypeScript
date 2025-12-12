---
description: "fromEvent() - Creation Function that converts DOM events and EventEmitter to Observable streams with automatic listener management and memory leak prevention"
---

# fromEvent() - Convert Events to Observable

`fromEvent()` is a Creation Function that converts event sources such as DOM events and Node.js EventEmitter into Observable streams.

## Overview

`fromEvent()` allows RxJS pipelines to handle event-based asynchronous processing. It automatically registers event listeners when subscribed and automatically removes listeners when unsubscribed, greatly reducing the risk of memory leaks.

**Signature**:
```typescript
function fromEvent<T>(
  target: any,
  eventName: string,
  options?: EventListenerOptions | ((...args: any[]) => T)
): Observable<T>
```

**Official Documentation**: [📘 RxJS Official: fromEvent()](https://rxjs.dev/api/index/function/fromEvent)

## Basic Usage

This is the simplest example of treating DOM events as Observable.

```typescript
import { fromEvent } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.subscribe(event => {
  console.log('Button clicked:', event);
});

// Event is emitted each time you click
```

## Important Characteristics

### 1. Automatic Listener Registration and Removal

`fromEvent()` registers an event listener upon subscription and automatically removes the listener upon unsubscription.

```typescript
import { fromEvent } from 'rxjs';

const clicks$ = fromEvent<MouseEvent>(document, 'click');

const subscription = clicks$.subscribe(event => {
  console.log('Click position:', event.clientX, event.clientY);
});

// Unsubscribe after 5 seconds (event listener is automatically removed)
setTimeout(() => {
  subscription.unsubscribe();
  console.log('Unsubscribed');
}, 5000);
```

> [!IMPORTANT]
> **Memory Leak Prevention**
>
> When `unsubscribe()` is called, `removeEventListener()` is automatically executed internally. This eliminates the need to manually remove listeners and greatly reduces the risk of memory leaks.

### 2. Cold Observable (Each Subscription Registers Independent Listener)

The Observable created by `fromEvent()` is a **Cold Observable**. Each subscription registers an independent event listener.

```typescript
import { fromEvent } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// Subscription 1 - Register listener A
clicks$.subscribe(() => console.log('Observer 1: Click'));

// Add subscription 2 after 1 second - Register listener B independently
setTimeout(() => {
  clicks$.subscribe(() => console.log('Observer 2: Click'));
}, 1000);

// Both listeners fire on a single click
// This proves that each subscription has an independent listener
```

> [!NOTE]
> **Cold Observable Proof**
>
> A new event listener is registered each time you subscribe and removed when you unsubscribe. This is a feature of Cold Observable. However, since the event source (e.g., DOM element) is external and shared, it also has the Hot property of "not receiving events before subscription".

### 3. TypeScript Type Support

Event types can be explicitly specified.

```typescript
import { fromEvent } from 'rxjs';

const input = document.createElement('input');
input.type = 'text';
document.body.appendChild(input);
const input$ = fromEvent<InputEvent>(input, 'input');

input$.subscribe(event => {
  // event type is InputEvent
  const target = event.target as HTMLInputElement;
  console.log('Input value:', target.value);
});
```

### 4. Cold Observable

`fromEvent()` is a **Cold Observable**. Each subscription initiates an independent execution.

```typescript
import { fromEvent } from 'rxjs';

const button = document.createElement('button');
button.innerText = "Subscribe";
document.body.appendChild(button);

const clicks$ = fromEvent(document, 'click');

// First subscription - event listener is added
clicks$.subscribe(() => console.log('Subscriber A'));

// Second subscription - another event listener is added
clicks$.subscribe(() => console.log('Subscriber B'));

// Both listeners fire when clicked once
// Output:
// Subscriber A
// Subscriber B
```

> [!NOTE]
> **Cold Observable Characteristics**:
> - Independent execution is initiated for each subscription
> - Each subscriber receives its own data stream
> - An independent event listener is registered for each subscription; unsubscribe automatically removes the listener
>
> See [Cold Observable and Hot Observable](/pt/guide/observables/cold-and-hot-observables) for more information.

## Practical Use Cases

### 1. Click Event Processing

Control button clicks and prevent consecutive clicks.

```typescript
import { fromEvent } from 'rxjs';
import { debounceTime, map } from 'rxjs';

const button = document.createElement('button');
button.innerText = "submit";
document.body.appendChild(button);

const clicks$ = fromEvent(button, 'click');

clicks$.pipe(
  debounceTime(300), // Ignore consecutive clicks within 300ms
  map(() => 'Submitting...')
).subscribe(message => {
  console.log(message);
  // API call processing, etc.
});
```

### 2. Real-time Form Input Validation

Stream input events and perform validation in real time.

```typescript
import { fromEvent } from 'rxjs';
import { map, debounceTime, distinctUntilChanged } from 'rxjs';

const label = document.createElement('label');
label.innerText = 'email: ';
const emailInput = document.createElement('input');
label.appendChild(emailInput);
document.body.appendChild(label);
const email$ = fromEvent<InputEvent>(emailInput, 'input');

email$.pipe(
  map(event => (event.target as HTMLInputElement).value),
  debounceTime(500), // Process 500ms after input stops
  distinctUntilChanged() // Only when value changes
).subscribe(email => {
  console.log('Validation target:', email);
  // Email validation processing
  validateEmail(email);
});

function validateEmail(email: string): void {
  const isValid = /^[^\s@]+@[^\s@]+\.[^\s@]+$/.test(email);
  console.log(isValid ? 'Valid email address' : 'Invalid email address');
}
```

### 3. Drag & Drop Implementation

Combine mouse events to implement drag & drop.

```typescript
import { fromEvent } from 'rxjs';
import { switchMap, takeUntil, map } from 'rxjs';

// Create draggable element
const element = document.createElement('div');
element.style.width = '100px';
element.style.height = '100px';
element.style.backgroundColor = '#333';
element.style.position = 'absolute'; // Set to absolute positioning
element.style.left = '50px'; // Initial position
element.style.top = '50px';
element.style.cursor = 'move'; // Draggable cursor
document.body.appendChild(element);

const mousedown$ = fromEvent<MouseEvent>(element, 'mousedown');
const mousemove$ = fromEvent<MouseEvent>(document, 'mousemove');
const mouseup$ = fromEvent<MouseEvent>(document, 'mouseup');

mousedown$.pipe(
  switchMap(startEvent => {
    // Record click position within element
    const startX = startEvent.clientX - element.offsetLeft;
    const startY = startEvent.clientY - element.offsetTop;

    return mousemove$.pipe(
      map(moveEvent => ({
        left: moveEvent.clientX - startX,
        top: moveEvent.clientY - startY
      })),
      takeUntil(mouseup$) // End on mouse up
    );
  })
).subscribe(({ left, top }) => {
  // Update element position
  element.style.left = `${left}px`;
  element.style.top = `${top}px`;
});
```

### 4. Monitoring Scroll Events

Used to track infinite scrolling and scroll position.

```typescript
import { fromEvent } from 'rxjs';
import { throttleTime, map } from 'rxjs';

const scroll$ = fromEvent(window, 'scroll');

scroll$.pipe(
  throttleTime(200), // Process only once every 200ms
  map(() => window.scrollY)
).subscribe(scrollPosition => {
  console.log('Scroll position:', scrollPosition);

  // Load additional content when reaching bottom of page
  if (scrollPosition + window.innerHeight >= document.body.scrollHeight - 100) {
    console.log('Load additional content');
    // loadMoreContent();
  }
});
```

## Using in Pipeline

`fromEvent()` is ideal for pipeline processing starting from event streams.

```typescript
import { fromEvent } from 'rxjs';
import { map, filter, scan } from 'rxjs';

const button = document.createElement('button');
button.innerText = "Counter";
document.body.appendChild(button);

const clicks$ = fromEvent(button, 'click');

clicks$.pipe(
  filter((event: Event) => {
    // Count only clicks while holding Shift key
    return (event as MouseEvent).shiftKey;
  }),
  scan((count, _) => count + 1, 0),
  map(count => `Click count: ${count}`)
).subscribe(message => console.log(message));
```

## Common Mistakes

### 1. Forgetting to Unsubscribe

#### ❌ Wrong - Forgetting to unsubscribe causes memory leaks

```typescript
import { fromEvent } from 'rxjs';

function setupEventListener() {
  const clicks$ = fromEvent(document, 'click');
  clicks$.subscribe(console.log); // Not unsubscribed!
}

setupEventListener();
```

#### ✅ Correct - Always unsubscribe

```typescript
import { fromEvent } from 'rxjs';
import { Subscription } from 'rxjs';

let subscription: Subscription;

function setupEventListener() {
  const clicks$ = fromEvent(document, 'click');
  subscription = clicks$.subscribe(console.log);
}

function cleanup() {
  if (subscription) {
    subscription.unsubscribe();
  }
}

setupEventListener();
// Call cleanup() when component is destroyed, etc.
```

> [!WARNING]
> **Beware of Memory Leaks**
>
> In SPA and component-based frameworks, be sure to unsubscribe when you destroy a component. If you forget to unsubscribe, event listeners will remain and cause memory leaks.

### 2. Duplicate Registration of Multiple Event Listeners

#### ❌ Wrong - Subscribing to the same event multiple times registers multiple listeners

```typescript
import { fromEvent } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.subscribe(() => console.log('Observer 1'));
clicks$.subscribe(() => console.log('Observer 2'));
// Both logs are displayed when clicked (two listeners are registered)
```

#### ✅ Correct - Multicast with share() as needed

```typescript
import { fromEvent } from 'rxjs';
import { share } from 'rxjs';

const clicks$ = fromEvent(document, 'click').pipe(share());

clicks$.subscribe(() => console.log('Observer 1'));
clicks$.subscribe(() => console.log('Observer 2'));
// One listener is shared
```

## Performance Considerations

Performance should be considered when handling events that fire at high frequency (scroll, mousemove, resize, etc.).

> [!TIP]
> **Optimization of High Frequency Events**:
> - `throttleTime()` - Process only once every certain period of time
> - `debounceTime()` - Process after input stops
> - `distinctUntilChanged()` - Process only when value changes

#### ❌ Performance Problem - Process on every resize

```typescript
import { fromEvent } from 'rxjs';

const resize$ = fromEvent(window, 'resize');

resize$.subscribe(() => {
  console.log('Resize processing'); // Heavy processing
});
```

#### ✅ Optimization - Process only once every 200ms

```typescript
import { fromEvent } from 'rxjs';
import { throttleTime } from 'rxjs';

const resize$ = fromEvent(window, 'resize');
resize$.pipe(
  throttleTime(200)
).subscribe(() => {
  console.log('Resize processing'); // Load reduction
});
```

## Related Creation Functions

| Function | Difference | Usage |
|----------|------|----------|
| **[from()](/pt/guide/creation-functions/basic/from)** | Convert from array/Promise | Stream data other than events |
| **[interval()](/pt/guide/creation-functions/basic/interval)** | Emit at regular intervals | Periodic processing needed |
| **fromEventPattern()** | Custom event registration | Custom event systems other than EventEmitter |

## Summary

- `fromEvent()` converts DOM events and EventEmitter to Observable
- Register listeners when subscribed, automatically delete when unsubscribed (prevents memory leaks)
- Works as Hot Observable
- Always perform unsubscribe to prevent memory leaks
- Optimize high frequency events with `throttleTime()` and `debounceTime()`

## Next Steps

- [interval() - Emit Values at Regular Intervals](/pt/guide/creation-functions/basic/interval)
- [timer() - Start Emitting After Delay](/pt/guide/creation-functions/basic/timer)
- [Return to Basic Creation Functions](/pt/guide/creation-functions/basic/)
