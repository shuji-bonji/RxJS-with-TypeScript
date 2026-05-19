---
description: "The sampleTime operator is an RxJS filtering operator that periodically samples the latest values of a stream at specified time intervals. It is ideal for periodic snapshot acquisition."
---

# sampleTime - get latest value periodically

The `sampleTime` operator periodically **samples** the latest value of the source Observable at **specified time intervals** and outputs it.
Like a periodic snapshot, it retrieves the most recent value at that point in time.

## 🔰 Basic Syntax and Usage

```ts
import { fromEvent } from 'rxjs';
import { sampleTime } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.pipe(
  sampleTime(2000)
).subscribe(() => {
  console.log('2Second-by-second samples');
});
```

**Flow of operation**:.
1. timer fires periodically every 2 seconds
2. output the latest click event if any at that time 3. output nothing if there is no value during the sample period
If there is no value during the sample period, nothing is output.

```ts
import { fromEvent } from 'rxjs';
import { sampleTime } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.pipe(
  sampleTime(2000)
).subscribe(() => {
  console.log('2Second-by-second samples');
});
```

> The above sample omits `fromEvent` unsubscription for simplicity of explanation. In real code, use `takeUntil(destroy$)`, `take(N)`, or `Subscription.unsubscribe()` to explicitly manage the lifecycle. More info: [Overcoming Difficulties: Lifecycle Management](/en/guide/overcoming-difficulties/lifecycle-management.md)

[🌐 Official RxJS documentation - `sampleTime`](https://rxjs.dev/api/operators/sampleTime)

## 💡 Typical utilization patterns

- **Periodic sensor data acquisition**: up-to-date temperature and location information every second
- **Real-time dashboard**: periodic status updates
- **Performance monitoring**: metrics collection at regular intervals
- **Game frame processing**: periodic sampling for FPS control

## 🧠 Practical code example 1: Periodic sampling of mouse position

This is an example of sampling the mouse position every second.


```ts
import { fromEvent } from 'rxjs';
import { sampleTime, map } from 'rxjs';

// UICreation
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Mouse position sampling (1(every second)';
container.appendChild(title);

const area = document.createElement('div');
area.style.width = '100%';
area.style.height = '300px';
area.style.border = '2px solid #4CAF50';
area.style.backgroundColor = '#f5f5f5';
area.style.display = 'flex';
area.style.alignItems = 'center';
area.style.justifyContent = 'center';
area.style.fontSize = '18px';
area.textContent = 'Move mouse within this area';
container.appendChild(area);

const output = document.createElement('div');
output.style.marginTop = '10px';
output.style.maxHeight = '150px';
output.style.overflow = 'auto';
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
container.appendChild(output);

let sampleCount = 0;

// Mouse move event
fromEvent<MouseEvent>(area, 'mousemove').pipe(
  map(event => ({
    x: event.offsetX,
    y: event.offsetY,
    timestamp: Date.now()
  })),
  sampleTime(1000) // 1Sampling every second
).subscribe(pos => {
  sampleCount++;
  const log = document.createElement('div');
  log.style.padding = '5px';
  log.style.borderBottom = '1px solid #eee';
  log.innerHTML = `
    <strong>Sample #${sampleCount}</strong>
    [${new Date(pos.timestamp).toLocaleTimeString()}]
    Position: (${pos.x}, ${pos.y})
  `;
  output.insertBefore(log, output.firstChild);

  // Maximum10Display up to
  while (output.children.length > 10) {
    output.removeChild(output.lastChild!);
  }
});
```

- If the mouse is continuously moved, only the most recent position at that point is sampled every second.
- If the mouse is not moved for one second, nothing is output for that period.

## 🎯 Practical Code Example 2: Real-time data dashboard

This is an example of periodically sampling sensor data and displaying it on a dashboard.

```ts
import { interval } from 'rxjs';
import { sampleTime, map } from 'rxjs';

// UICreation
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Sensor Monitoring Dashboard';
container.appendChild(title);

const dashboard = document.createElement('div');
dashboard.style.display = 'grid';
dashboard.style.gridTemplateColumns = '1fr 1fr';
dashboard.style.gap = '10px';
dashboard.style.marginTop = '10px';
container.appendChild(dashboard);

// Dashboard card creation
function createCard(label: string, unit: string) {
  const card = document.createElement('div');
  card.style.padding = '20px';
  card.style.border = '2px solid #2196F3';
  card.style.borderRadius = '8px';
  card.style.backgroundColor = '#E3F2FD';

  const labelDiv = document.createElement('div');
  labelDiv.textContent = label;
  labelDiv.style.fontSize = '14px';
  labelDiv.style.color = '#666';
  card.appendChild(labelDiv);

  const valueDiv = document.createElement('div');
  valueDiv.style.fontSize = '32px';
  valueDiv.style.fontWeight = 'bold';
  valueDiv.style.marginTop = '10px';
  valueDiv.textContent = '--';
  card.appendChild(valueDiv);

  const unitDiv = document.createElement('div');
  unitDiv.textContent = unit;
  unitDiv.style.fontSize = '14px';
  unitDiv.style.color = '#666';
  card.appendChild(unitDiv);

  dashboard.appendChild(card);
  return valueDiv;
}

const tempValue = createCard('Temperature', '°C');
const humidityValue = createCard('Humidity', '%');
const pressureValue = createCard('Barometric pressure', 'hPa');
const lightValue = createCard('Illuminance', 'lux');

// Sensor data stream (updated every100msUpdated every)
const sensorData$ = interval(100).pipe(
  map(() => ({
    temperature: (20 + Math.random() * 10).toFixed(1),
    humidity: (40 + Math.random() * 40).toFixed(1),
    pressure: (1000 + Math.random() * 30).toFixed(1),
    light: Math.floor(Math.random() * 1000)
  }))
);

// 2Sampled and updated dashboard every second
sensorData$.pipe(
  sampleTime(2000)
).subscribe(data => {
  tempValue.textContent = data.temperature;
  humidityValue.textContent = data.humidity;
  pressureValue.textContent = data.pressure;
  lightValue.textContent = data.light.toString();

  // Animation effect
  [tempValue, humidityValue, pressureValue, lightValue].forEach(elem => {
    elem.style.color = '#2196F3';
    setTimeout(() => {
      elem.style.color = 'black';
    }, 500);
  });
});
```

- The sensor data is updated every 100ms, but the dashboard is updated with sampled values every 2 seconds.
- Performance can be optimized by displaying high frequency data streams at appropriate intervals.

## 🆚 Comparison with similar operators

### sampleTime vs throttleTime vs auditTime

```ts
import { interval } from 'rxjs';
import { sampleTime, throttleTime, auditTime, take } from 'rxjs';

const source$ = interval(300).pipe(take(10)); // 0, 1, 2, 3, ...

// sampleTime: 1Sampling of the latest value at that moment every second
source$.pipe(
  sampleTime(1000)
).subscribe(val => console.log('sampleTime:', val));
// Output example: 2, 5, 8(A snapshot every second)1Snapshot every second)

// throttleTime: After outputting the first value,1seconds after the first value is output
source$.pipe(
  throttleTime(1000)
).subscribe(val => console.log('throttleTime:', val));
// Output example: 0, 3, 6, 9(first value of each period)

// auditTime: Output the last value of the period1seconds after the first value, the last value of the period is output.
source$.pipe(
  auditTime(1000)
).subscribe(val => console.log('auditTime:', val));
// Output example: 2, 5, 8(the last value of each period)
```

```ts
import { fromEvent } from 'rxjs';
import { sampleTime } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.pipe(
  sampleTime(2000)
).subscribe(() => {
  console.log('2Second-by-second samples');
});
```

**visual differences**:.

Input: --|1|2|3|---|4|5|6|---|7|8|9|
      0s  1s      2s      3s

sampleTime(1s):  -------|3|-------|6|-------|9|
                 (Periodic sampling)

throttleTime(1s): |1|--------------|4|--------------|7|
                  (Ignored during the period through the beginning)

auditTime(1s):    -------|3|-------|6|-------|9|
                  (Last value at end of period)
```

## ⚠️ Notes

### 1. no value during the sample period

If there are no new values at sample timing, nothing is output.

```

```ts
import { fromEvent } from 'rxjs';
import { sampleTime } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.pipe(
  sampleTime(2000)
).subscribe(() => {
  console.log('Samples taken');
});
// 2During a period of seconds1If no clicks are made, nothing is output
```

### 2. wait for the first sample timing

The `sampleTime` will not output anything until the specified time has elapsed.

```ts
import { interval } from 'rxjs';
import { sampleTime } from 'rxjs';

interval(100).pipe(
  sampleTime(1000)
).subscribe(console.log);
// First value is output after1seconds after the first value is output
```

### 3. completion timing

When a source completes, completion is not propagated until the next sample timing.

```ts
import { of } from 'rxjs';
import { sampleTime, delay } from 'rxjs';

of(1, 2, 3).pipe(
  delay(100),
  sampleTime(1000)
).subscribe({
  next: console.log,
  complete: () => console.log('Completed')
});
// 1seconds later: 3
// 1seconds later: Completed
```

### 4. memory usage

Memory efficiency is good because only one latest value is kept internally.

```ts
import { interval } from 'rxjs';
import { sampleTime } from 'rxjs';

// High frequency stream (10msper second)
interval(10).pipe(
  sampleTime(1000) // 1Sampling every second
).subscribe(console.log);
// Memory holds only the most recent1Only the two most recent values are retained in memory
```

## 💡 Difference from sample

`sample` uses another Observable as a trigger, while `sampleTime` uses a fixed time interval.

```ts
import { fromEvent } from 'rxjs';
import { sampleTime } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.pipe(
  sampleTime(2000)
).subscribe(() => {
  console.log('2Second-by-second samples');
});
```


## 📚 Related Operators

- **[sample](https://rxjs.dev/api/operators/sample)** - Sampling another Observable as a trigger (official documentation)
- **[throttleTime](. /throttleTime)** - Get the first value at the start of the period.
- **[auditTime](. /auditTime)** - get the last value at the end of the period
- **[debounceTime](. /debounceTime)** - issue value after a pause

## Summary

The `sampleTime` operator periodically samples the latest value at the specified time interval.

- ✅ Ideal for taking periodic snapshots
- ✅ Useful for thinning out high frequency streams
- ✅ Memory efficient (keeps only one latest value)
- ✅ Ideal for dashboards and monitoring
- ⚠️ No value during the sample period, nothing is output
- ⚠️ There is a wait time until the first sample
- ⚠️ Completion is propagated at next sample timing
