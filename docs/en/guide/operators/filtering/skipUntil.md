---
description: The skipUntil operator skips all values from the original Observable until another Observable emits a value, then outputs values normally. It is useful for time-based delayed start or processing after a specific event occurs.
---

# skipUntil - Skip Until Another Observable Fires

The `skipUntil` operator **skips all values from the source Observable** until a specified Observable (notification trigger) emits its first value. After the notification trigger emits, subsequent values are output normally.


## 🔰 Basic Syntax and Usage

```ts
import { interval, timer } from 'rxjs';
import { skipUntil } from 'rxjs';

const source$ = interval(500); // Emit value every 0.5 seconds
const notifier$ = timer(2000); // Emit value after 2 seconds

source$.pipe(
  skipUntil(notifier$)
).subscribe(console.log);
// Output: 4, 5, 6, 7, 8, ...
// (First 2 seconds values 0, 1, 2, 3 are skipped)
```

**Flow of operation**:
1. `source$` emits 0, 1, 2, 3 → all skipped
2. After 2 seconds, `notifier$` emits a value
3. Subsequent `source$` values (4, 5, 6, ...) are output normally

[🌐 RxJS Official Documentation - `skipUntil`](https://rxjs.dev/api/operators/skipUntil)


## 🆚 Contrast with takeUntil

`skipUntil` and `takeUntil` have contrasting behavior.

```ts
import { interval, timer } from 'rxjs';
import { skipUntil, takeUntil } from 'rxjs';

const source$ = interval(500); // Emit value every 0.5 seconds
const notifier$ = timer(2000); // Emit value after 2 seconds

// takeUntil: Take values until notification
source$.pipe(
  takeUntil(notifier$)
).subscribe(console.log);
// Output: 0, 1, 2, 3 (stops after 2 seconds)

// skipUntil: Skip values until notification
source$.pipe(
  skipUntil(notifier$)
).subscribe(console.log);
// Output: 4, 5, 6, 7, ... (starts after 2 seconds)
```

| Operator | Behavior | Completion Timing |
|---|---|---|
| `takeUntil(notifier$)` | **Take** values until notification | Auto-completes when notified |
| `skipUntil(notifier$)` | **Skip** values until notification | When source stream completes |


## 💡 Typical Usage Patterns

1. **Start Processing Data After User Authentication**
   ```ts
   import { interval, Subject } from 'rxjs';
   import { skipUntil } from 'rxjs';

   const authenticated$ = new Subject<void>();
   const dataStream$ = interval(1000);

   // Skip data until authentication completes
   dataStream$.pipe(
     skipUntil(authenticated$)
   ).subscribe(data => {
     console.log(`Processing data: ${data}`);
   });

   // Authentication completes after 3 seconds
   setTimeout(() => {
     console.log('Authentication complete!');
     authenticated$.next();
   }, 3000);
   // After 3 seconds, outputs "Processing data: 3", "Processing data: 4", ...
   ```

2. **Start Event Processing After Initial Load Completes**
   ```ts
   import { fromEvent, BehaviorSubject } from 'rxjs';
   import { filter, skipUntil } from 'rxjs';

   const appReady$ = new BehaviorSubject<boolean>(false);
   const button = document.createElement('button');
   button.textContent = 'Click';
   document.body.appendChild(button);

   const clicks$ = fromEvent(button, 'click');

   // Ignore clicks until app is ready
   clicks$.pipe(
     skipUntil(appReady$.pipe(filter(ready => ready)))
   ).subscribe(() => {
     console.log('Click processed');
   });

   // App ready after 2 seconds
   setTimeout(() => {
     console.log('App ready');
     appReady$.next(true);
   }, 2000);
   ```

3. **Timer-Based Delayed Start**
   ```ts
   import { fromEvent, timer } from 'rxjs';
   import { skipUntil, scan } from 'rxjs';

   const button = document.createElement('button');
   button.textContent = 'Count';
   document.body.appendChild(button);

   const clicks$ = fromEvent(button, 'click');
   const startTime$ = timer(3000); // After 3 seconds

   // Don't count clicks until 3 seconds pass
   clicks$.pipe(
     skipUntil(startTime$),
     scan(count => count + 1, 0)
   ).subscribe(count => {
     console.log(`Count: ${count}`);
   });

   console.log('Counting starts after 3 seconds...');
   ```


## 🧠 Practical Code Example (Game Countdown)

Example of ignoring clicks during the countdown before the game starts and enabling clicks after the countdown ends.

```ts
import { fromEvent, timer, interval } from 'rxjs';
import { skipUntil, take, scan } from 'rxjs';

// Create UI elements
const container = document.createElement('div');
document.body.appendChild(container);

const countdown = document.createElement('div');
countdown.style.fontSize = '24px';
countdown.style.marginBottom = '10px';
countdown.textContent = 'Counting down...';
container.appendChild(countdown);

const button = document.createElement('button');
button.textContent = 'Click!';
button.disabled = true;
container.appendChild(button);

const scoreDisplay = document.createElement('div');
scoreDisplay.style.marginTop = '10px';
scoreDisplay.textContent = 'Score: 0';
container.appendChild(scoreDisplay);

// Countdown (3 seconds)
const countdownTimer$ = interval(1000).pipe(take(3));
countdownTimer$.subscribe({
  next: (n) => {
    countdown.textContent = `Starting in ${3 - n} seconds...`;
  },
  complete: () => {
    countdown.textContent = 'Game Start!';
    button.disabled = false;
  }
});

// Game start notification
const gameStart$ = timer(3000);

// Click events (skip until game starts)
const clicks$ = fromEvent(button, 'click');

clicks$.pipe(
  skipUntil(gameStart$),
  scan(score => score + 10, 0)
).subscribe(score => {
  scoreDisplay.textContent = `Score: ${score}`;
});
```

In this code, clicks are ignored during the 3-second countdown, and only clicks after the countdown ends are reflected in the score.


## 🎯 Difference Between skip and skipUntil

```ts
import { interval, timer } from 'rxjs';
import { skip, skipUntil } from 'rxjs';

const source$ = interval(500);

// skip: Skip first N values by count
source$.pipe(
  skip(3)
).subscribe(console.log);
// Output: 3, 4, 5, 6, ...

// skipUntil: Skip until another Observable fires
source$.pipe(
  skipUntil(timer(1500))
).subscribe(console.log);
// Output: 3, 4, 5, 6, ... (same result, but different control method)
```

| Operator | Skip Condition | Use Case |
|---|---|---|
| `skip(n)` | Skip first n by count | Fixed count skip |
| `skipWhile(predicate)` | Skip while condition is met | Condition-based skip |
| `skipUntil(notifier$)` | Skip until another Observable fires | Event/time-based skip |


## 📋 Type-Safe Usage

Type-safe implementation example utilizing TypeScript generics.

```ts
import { Observable, Subject, fromEvent } from 'rxjs';
import { skipUntil, map } from 'rxjs';

interface GameState {
  status: 'waiting' | 'ready' | 'playing' | 'finished';
}

interface ClickEvent {
  timestamp: number;
  x: number;
  y: number;
}

class Game {
  private gameReady$ = new Subject<void>();
  private state: GameState = { status: 'waiting' };

  startGame(element: HTMLElement): Observable<ClickEvent> {
    const clicks$ = fromEvent<MouseEvent>(element, 'click').pipe(
      map(event => ({
        timestamp: Date.now(),
        x: event.clientX,
        y: event.clientY
      } as ClickEvent)),
      skipUntil(this.gameReady$)
    );

    // Notify ready
    setTimeout(() => {
      this.state = { status: 'ready' };
      this.gameReady$.next();
      console.log('Game ready!');
    }, 2000);

    return clicks$;
  }
}

// Usage example
const game = new Game();
const canvas = document.createElement('div');
canvas.style.width = '300px';
canvas.style.height = '200px';
canvas.style.border = '1px solid black';
canvas.textContent = 'Please click';
document.body.appendChild(canvas);

game.startGame(canvas).subscribe(click => {
  console.log(`Click position: (${click.x}, ${click.y})`);
});
```


## 🔄 Combining skipUntil and takeUntil

To take values only for a specific period, combine both.

```ts
import { interval, timer } from 'rxjs';
import { skipUntil, takeUntil } from 'rxjs';

const source$ = interval(500);
const start$ = timer(2000); // Start after 2 seconds
const stop$ = timer(5000);  // Stop after 5 seconds

source$.pipe(
  skipUntil(start$), // Skip until 2 seconds
  takeUntil(stop$)   // Stop at 5 seconds
).subscribe({
  next: console.log,
  complete: () => console.log('Complete')
});
// Output: 4, 5, 6, 7, 8, 9, Complete
// (Only values between 2-5 seconds)
```

**Timeline**:
```
0s    1s    2s    3s    4s    5s
|-----|-----|-----|-----|-----|
0  1  2  3  4  5  6  7  8  9  10
      ↑           ↑
   skip start  take end
   (from 4)    (to 9)
```


## ⚠️ Common Mistakes

> [!IMPORTANT]
> `skipUntil` is only valid for the **first emission** of the notification Observable. Second and subsequent emissions are ignored.

### Wrong: Notification Observable Fires Multiple Times

```ts
import { interval, Subject } from 'rxjs';
import { skipUntil } from 'rxjs';

const source$ = interval(500);
const notifier$ = new Subject<void>();

source$.pipe(
  skipUntil(notifier$)
).subscribe(console.log);

// ❌ Bad example: Calling next multiple times only takes effect once
setTimeout(() => notifier$.next(), 1000);
setTimeout(() => notifier$.next(), 2000); // This is meaningless
```

### Correct: Understand Only First Emission is Valid

```ts
import { interval, Subject } from 'rxjs';
import { skipUntil } from 'rxjs';

const source$ = interval(500);
const notifier$ = new Subject<void>();

source$.pipe(
  skipUntil(notifier$)
).subscribe(console.log);

// ✅ Good example: Call next only once
setTimeout(() => {
  console.log('Skip ended');
  notifier$.next();
  notifier$.complete(); // Best practice to complete
}, 1000);
```


## 🎓 Summary

### When to Use skipUntil
- ✅ When you want to start processing after a specific event occurs
- ✅ When you want to enable user operations after initialization completes
- ✅ When time-based delayed start is needed
- ✅ When you want to start data processing after authentication completes

### Combination with takeUntil
- ✅ When you want to take values only for a specific period (skipUntil + takeUntil)

### Notes
- ⚠️ Only the first emission of the notification Observable is valid
- ⚠️ If the notification Observable doesn't emit, all values continue to be skipped
- ⚠️ Subscription is maintained until the source stream completes


## 🚀 Next Steps

- **[skip](/en/guide/operators/filtering/skip)** - Learn how to skip first N values
- **[take](/en/guide/operators/filtering/take)** - Learn how to take first N values
- **[takeUntil](../utility/takeUntil)** - Learn how to take values until another Observable fires
- **[filter](/en/guide/operators/filtering/filter)** - Learn how to filter based on conditions
- **[Filtering Operator Practical Examples](/en/guide/operators/filtering/practical-use-cases)** - Learn real use cases
