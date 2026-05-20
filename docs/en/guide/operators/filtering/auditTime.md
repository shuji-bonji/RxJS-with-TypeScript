---
description: "auditTime is an RxJS filtering operator that waits for a specified time after a value is issued and outputs the last value within that period. It is best used when you want to periodically sample the latest state on high frequency events such as scroll position tracking, window resizing, mouse movement, etc. It is important to understand the difference from throttleTime and debounceTime and use them appropriately."
---

# auditTime - issue last value after specified time

The `auditTime` operator waits for a **specified time** after a value is issued and outputs the **last value** within that time period. It then waits for the next value.

## 🔰 Basic Syntax and Usage

```ts
import { fromEvent } from 'rxjs';
import { auditTime } from 'rxjs';

fromEvent(document, 'click').pipe(
  auditTime(1000)
).subscribe(() => console.log('Click!'));
```

**Flow of operation**: 1.
1. first click occurs
2. wait 1 second (clicks during this time are recorded but not output) 3.
3. output the last click after 1 second 4. wait for the next click
Wait for next click

[🌐 RxJS Official Documentation - `auditTime`](https://rxjs.dev/api/operators/auditTime)

## 🆚 Contrast with throttleTime

`throttleTime` and `auditTime` are similar, but differ in the values they output.

```ts
import { interval } from 'rxjs';
import { throttleTime, auditTime, take } from 'rxjs';

const source$ = interval(300).pipe(take(10)); // 0, 1, 2, 3, 4, 5, 6, 7, 8, 9

// throttleTime: Output first value
source$.pipe(
  throttleTime(1000)
).subscribe(console.log);
// Output: 0, 4, 8(the first value for each period)

// auditTime: Output last value
source$.pipe(
  auditTime(1000)
).subscribe(console.log);
// Output: 3, 6, 9(last value of each period)
```

**Timeline Comparison**:.

```
Source:     0--1--2--3--4--5--6--7--8--9--|
            |        |        |
throttle:   0--------4--------8------------|
            (First)   (First)   (First)

audit:      -------3--------6--------9----|
                  (Last)   (Last)   (Last)
```

```ts
import { fromEvent } from 'rxjs';
import { auditTime } from 'rxjs';

fromEvent(document, 'click').pipe(
  auditTime(1000)
).subscribe(() => console.log('Click!'));
```

## 💡 Typical utilization pattern

1. **Optimize window resizing**.


```ts
   import { fromEvent } from 'rxjs';
   import { auditTime } from 'rxjs';

   fromEvent(window, 'resize').pipe(
     auditTime(200) // 200msGet the latest size in the interval
   ).subscribe(() => {
     console.log(`Window size: ${window.innerWidth}x${window.innerHeight}`);
   });
   ```

2. **Track scroll position**
   ```ts
   import { fromEvent } from 'rxjs';
   import { auditTime, map } from 'rxjs';

   fromEvent(window, 'scroll').pipe(
     auditTime(100),
     map(() => ({
       scrollY: window.scrollY,
       scrollX: window.scrollX
     }))
   ).subscribe(position => {
     console.log(`Scroll position: Y=${position.scrollY}, X=${position.scrollX}`);
   });
   ```

3. **Smooth drag movement**
   ```ts
   import { fromEvent } from 'rxjs';
   import { auditTime, map, takeUntil, switchMap } from 'rxjs';

   // Create draggable elements
   const box = document.createElement('div');
   box.style.width = '100px';
   box.style.height = '100px';
   box.style.backgroundColor = '#3498db';
   box.style.position = 'absolute';
   box.style.cursor = 'move';
   box.style.left = '100px';
   box.style.top = '100px';
   box.textContent = 'Dragging';
   box.style.display = 'flex';
   box.style.alignItems = 'center';
   box.style.justifyContent = 'center';
   box.style.color = 'white';
   document.body.appendChild(box);

   const mouseDown$ = fromEvent<MouseEvent>(box, 'mousedown');
   const mouseMove$ = fromEvent<MouseEvent>(document, 'mousemove');
   const mouseUp$ = fromEvent<MouseEvent>(document, 'mouseup');

   // Implement dragging operations
   mouseDown$.pipe(
     switchMap(startEvent => {
       const startX = startEvent.clientX - box.offsetLeft;
       const startY = startEvent.clientY - box.offsetTop;

       return mouseMove$.pipe(
         auditTime(16), // Approx.60FPS(with16msUpdate position in
         map(moveEvent => ({
           x: moveEvent.clientX - startX,
           y: moveEvent.clientY - startY
         })),
         takeUntil(mouseUp$)
       );
     })
   ).subscribe(position => {
     box.style.left = `${position.x}px`;
     box.style.top = `${position.y}px`;
   });
   ```

## 🧠 Practical Code Example (Mouse Tracking)

This is an example of tracking mouse movements and displaying the latest position at regular intervals.

```

ts
import { fromEvent } from 'rxjs';
import { auditTime, map } from 'rxjs';

// Create UI elements
const container = document.createElement('div');
container.style.height = '300px';
container.style.border = '2px solid #3498db';
container.style.padding = '20px';
container.style.position = 'relative';
container.textContent = 'Please move your mouse within this area';
document.body.appendChild(container);

const positionDisplay = document.createElement('div');
positionDisplay.style.marginTop = '10px';
positionDisplay.style.fontFamily = 'monospace';
document.body.appendChild(positionDisplay);

const dot = document.createElement('div');
dot.style.width = '10px';
dot.style.height = '10px';
dot.style.borderRadius = '50%';
dot.style.backgroundColor = '#e74c3c';
dot.style.position = 'absolute';
dot.style.display = 'none';
container.appendChild(dot);

// Mouse move event
fromEvent\<MouseEvent>(container, 'mousemove').pipe(
  map(event => {
    const rect = container.getBoundingClientRect();
    return {
      x: event.clientX - rect.left,.
      y: event.clientY - rect.top
    };
  }),
  auditTime(100) // get latest position every 100ms
).subscribe(position => {
  positionDisplay.textContent = `Latest position (every 100ms): X=${position.x.toFixed(0)}, Y=${position.y.toFixed(0)}`;

  // move dot to latest position
  dot.style.left = `${position.x - 5}px`;
  dot.style.top = `${position.y - 5}px`;
  dot.style.display = 'block';
});

```

This code will only retrieve and display the latest position for each mouse movement, even if the mouse moves frequently,100msThis code acquires and displays only the latest position for each mouse movement.

## 🎯 debounceTime Difference from

`auditTime` and `debounceTime` is that**both output the last value**but the timing is completely different,**The timing is completely different.**The

### The crucial difference

| Operator | operation | Use |
|---|---|---|
| `auditTime(ms)` | When a value comes in**msAlways output after**(even if input continues) | Periodic sampling |
| `debounceTime(ms)` | **After input stops**msOutput later | Wait for input completion |

### Specific examples：Difference in search input

```

ts
import { fromEvent } from 'rxjs';
import { auditTime, debounceTime } from 'rxjs';

const input = document.createElement('input');
input.placeholder = 'Search word input';
document.body.appendChild(input);

// auditTime: search is executed every 300ms even while inputting
fromEvent(input, 'input').pipe(
  auditTime(300)
).subscribe(() => {
  console.log('auditTime → Search:', input.value); }
});

// debounceTime: wait 300ms after input stops, then execute search
fromEvent(input, 'input').pipe(
  debounceTime(300)
).subscribe(() => {
  console.log('debounceTime → Search:', input.value);
});

```

### Differences seen in timeline

If a user clicks on the "ab"→"abc"→"abcdand typing quickly:

```

Input event: a--b--c--d------------|
              ↓↓
auditTime: ------c-----d----------|
            (after 300 ms) (after 300 ms)
            → Search for "abc", search for "abcd" (2 times total)

debounceTime: --------------------d-|
                              (300ms after stop)
            → Search for "abcd" (only once in total)

```

**Easy to remember**:
- **`auditTime`**: Regularly audited (auditAudit ( )"→ Always check at regular intervals
- **`debounceTime`**: "Wait untildebounceWait until the system is quiet.→ Wait until it is quiet.

### Practical Usage

```

ts
// ✅ auditTime if appropriate
// - Tracking scroll position (we want to get it periodically even if we are scrolling all the time)
fromEvent(window, 'scroll').pipe(
  auditTime(100) // get the latest position every 100ms
).subscribe(/* ... */);

// ✅ if debounceTime is appropriate
// - search box (want to search after input is complete)
fromEvent(searchInput, 'input').pipe(
  debounceTime(300) // wait 300ms after input stops
).subscribe(/* ... */);

```

## 📋 Type-safe usage

TypeScript This is an example of a type-safe implementation that utilizes generics in

```

ts
import { Observable, fromEvent } from 'rxjs';
import { auditTime, map } from 'rxjs';

interface MousePosition {
  x: number;
  y: number;
  timestamp: number; }
}

function trackMousePosition(
  element: HTMLElement,
  intervalMs: number
): Observable\<MousePosition> {
  return fromEvent\<MouseEvent>(element, 'mousemove').pipe(
    auditTime(intervalMs), map(event => ({ intervalMs: number ))
    map(event => ({
      x: event.clientX, event.
      y: event.clientY,
      timestamp: Date.now()
    } as MousePosition))
  );
}

// Example usage
const canvas = document.createElement('div');
canvas.style.width = '400px';
canvas.style.height = '300px';
canvas.style.border = '1px solid black';
document.body.appendChild(canvas);

trackMousePosition(canvas, 200).subscribe(position => {
  console.log(`position: (${position.x}, ${position.y}) at ${position.timestamp}`);
});

```

## 🔄 auditTime and throttleTime Combination of

In certain scenarios, both can be combined.

```

ts
import { interval } from 'rxjs';
import { throttleTime, auditTime, take } from 'rxjs';

const source$ = interval(100).pipe(take(50));

// order of throttleTime → auditTime
source$.pipe(
  throttleTime(1000), // pass the first value through every second
  auditTime(500) // then wait 500ms and output last value
).subscribe(console.log);

```ts
import { fromEvent } from 'rxjs';
import { auditTime } from 'rxjs';

fromEvent(document, 'click').pipe(
  auditTime(1000)
).subscribe(() => console.log('Click!'));
```

ts
import { fromEvent } from 'rxjs';
import { auditTime } from 'rxjs';

// Create search input field
const input = document.createElement('input');
input.type = 'text';
input.placeholder = 'search...' ;
document.body.appendChild(input);

// ❌ Bad example: use auditTime for search input
fromEvent(input, 'input').pipe(
  auditTime(300) // search is performed every 300ms while inputting
).subscribe(() => {
  console.log('search executed');
});


```ts
import { fromEvent } from 'rxjs';
import { auditTime } from 'rxjs';

fromEvent(document, 'click').pipe(
  auditTime(1000)
).subscribe(() => console.log('Click!'));
```

ts
import { fromEvent } from 'rxjs';
import { debounceTime } from 'rxjs';

// Create search input field
const input = document.createElement('input');
input.type = 'text';
input.placeholder = 'search...' ;
document.body.appendChild(input);

// ✅ Good example: use debounceTime for search input
fromEvent(input, 'input').pipe(
  debounceTime(300) // wait 300ms after input stops before searching
).subscribe(() => {
  console.log('Search executed', input.value);
});


## 🎓 Summary

### When auditTime should be used
- ✅ If you need up-to-date values at regular intervals
- ✅ High frequency events such as scrolling, resizing, mouse movement, etc.
- ✅ When periodic sampling is required
- ✅ When you want to reflect the latest status

### When should throttleTime be used?
- ✅ When an immediate response is needed
- ✅ If you want to start processing with the first value
- ✅ Prevention of button mashing

### When debounceTime should be used
- When you want to wait for input completion
- ✅ Search, autocomplete
- ✅ Wait until the user stops typing

### Notes.
- ⚠️ `auditTime` outputs only the last value in the period (intermediate values are discarded)
- ⚠️ Setting a short interval doesn't work very well
- ⚠️ `throttleTime` or `debounceTime` may be more appropriate for some applications

## 🚀 Next Steps

- **[throttleTime](./throttleTime)** - learn how to pass the first value through
- **[debounceTime](./debounceTime)** - learn how to issue a value after input stops
- **[filter](./filter)** - learn how to filter based on conditions
- **[filtering-operator-practical-use-cases](./practical-use-cases)** - learn how to use real use cases
