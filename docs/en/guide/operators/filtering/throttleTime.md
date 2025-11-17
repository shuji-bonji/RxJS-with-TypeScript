---
description: The throttleTime operator efficiently thins out high frequency events by allowing only the first value to pass within a specified time interval and ignoring subsequent values. It is ideal for real-time event optimization such as scrolling or mouse movement.
titleTemplate: ':title'
---

# throttleTime - Pass Through the First Value and Ignore New Values for the Specified Time

The `throttleTime` operator passes through the first value emitted and ignores subsequent values emitted within a specified time interval.
It does not emit the latest value at regular intervals, but rather **only passes through the first value it receives and ignores subsequent values during that period**.

This is useful for thinning out streams that fire frequently, such as scroll events and mouse movement events.


## 🔰 Basic Syntax and Usage

```ts
import { fromEvent } from 'rxjs';
import { throttleTime } from 'rxjs';

fromEvent(document, 'click')
  .pipe(throttleTime(2000))
  .subscribe(() => console.log('Clicked!'));

```

- Receives only the first click event every 2 seconds and ignores subsequent clicks.

[🌐 RxJS Official Documentation - `throttleTime`](https://rxjs.dev/api/operators/throttleTime)


## 💡 Typical Usage Patterns

- Optimization of event handling for scrolling and mouse movement
- Prevention of multiple submissions due to consecutive button presses
- Real-time data stream thinning


## 🧠 Practical Code Example (with UI)

When the mouse is moved, position information is displayed every 100 milliseconds.

```ts
import { fromEvent } from 'rxjs';
import { throttleTime, map } from 'rxjs';

// Create output area
const container = document.createElement('div');
container.style.height = '200px';
container.style.border = '1px solid #ccc';
container.style.padding = '10px';
container.textContent = 'Please move your mouse within this area';
document.body.appendChild(container);

const positionDisplay = document.createElement('div');
positionDisplay.style.marginTop = '10px';
document.body.appendChild(positionDisplay);

// Mouse move event
fromEvent<MouseEvent>(container, 'mousemove').pipe(
  map(event => ({
    x: event.clientX,
    y: event.clientY
  })),
  throttleTime(100)
).subscribe(position => {
  positionDisplay.textContent = `Mouse position: X=${position.x}, Y=${position.y}`;
});
```

- Limits frequently fired mouse movement events to every 100ms and displays only the most recent position.
