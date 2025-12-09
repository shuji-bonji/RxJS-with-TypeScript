---
description: Task-Klassifizierung (synchron, Microtask, Macrotask) und RxJS-Scheduler. JavaScript Event Loop, setTimeout, Promise und deren Ausführungsreihenfolge verstehen.
---

# Grundlagen zu Tasks und Schedulern

## Was ist synchrone Verarbeitung
Synchrone Verarbeitung wird sofort in der Reihenfolge ausgeführt, in der der Code geschrieben wurde, und geht nicht zum nächsten Vorgang über, bis der vorherige abgeschlossen ist.

#### Beispiel
```ts
console.log('A');
console.log('B');
console.log('C');

// Ausgabe:
// A
// B
// C
```


## Was ist asynchrone Verarbeitung
Asynchrone Verarbeitung wird nicht sofort ausgeführt, sondern nach Abschluss der aktuellen synchronen Verarbeitung.
Es gibt "Macrotasks" und "Microtasks" bei der asynchronen Verarbeitung.


## Macrotasks
- Tasks, die im nächsten Zyklus des Event Loops ausgeführt werden.
- Beispiele: `setTimeout`, `setInterval`, Browser-Events

#### Ausführungsbeispiel
```ts
console.log('Start');
setTimeout(() => console.log('Macro Task'), 0);
console.log('End');

// Ausgabe:
// Start
// End
// Micro Task
```

### Zuordnung in RxJS
- `asyncScheduler`
  - Verwendet intern `setTimeout`
  - Funktioniert als Macrotask

```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

of('Hello')
  .pipe(observeOn(asyncScheduler))
  .subscribe(console.log);

// Ausgabe:
// Hello
```


## Microtasks
- Tasks, die unmittelbar nach Abschluss des aktuellen Tasks und vor Beginn des nächsten Tasks ausgeführt werden.
- Beispiele: `Promise.then`, `queueMicrotask`

#### Ausführungsbeispiel
```ts
console.log('Start');
Promise.resolve().then(() => console.log('Micro Task'));
console.log('End');

// Ausgabe:
// Start
// End
// Micro Task
```

### Zuordnung in RxJS
- `asapScheduler`
  - Verwendet intern `Promise.resolve().then()`
  - Funktioniert als Microtask

```ts
import { of, asapScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

of('Hi')
  .pipe(observeOn(asapScheduler))
  .subscribe(console.log);

// Ausgabe:
// Hi
```


## Synchrone Tasks
- Normaler Code, der sofort ausgeführt wird.

### Zuordnung in RxJS
- `queueScheduler`
  - Erscheint synchron, ermöglicht aber feinere Steuerung durch Task-Queuing.

```ts
import { of, queueScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

of('Now')
  .pipe(observeOn(queueScheduler))
  .subscribe(console.log);

// Ausgabe:
// Now
```


## Zusammenfassung der Ausführungsreihenfolge

#### Codebeispiel
```ts
console.log('1');

setTimeout(() => console.log('2 (setTimeout)'), 0);
Promise.resolve().then(() => console.log('3 (Promise)'));

console.log('4');

// Ausgabe:
// 1
// 4
// 3 (Promise) 👈 Microtask
// 2 (setTimeout) 👈 Macrotask
```


## Zuordnungstabelle Tasks und RxJS-Scheduler

| Typ         | Beispiel                          | RxJS-Scheduler  |
|--------------|------------------------------|---------------------|
| Synchrone Verarbeitung     | Normaler Code                  | `queueScheduler`    |
| Microtask | Promise.then, queueMicrotask | `asapScheduler`     |
| Macrotask | setTimeout, setInterval      | `asyncScheduler`    |
