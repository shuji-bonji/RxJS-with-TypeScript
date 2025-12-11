---
description: Basisuitleg over de classificatie van taken (synchroon・microtask・macrotask) en hun relatie met RxJS-schedulers. Begrijp het JavaScript event loop-mechanisme, verschillen in uitvoervolgorde, en de implementatie en werking van setTimeout, Promise, queueMicrotask, etc., en pas deze kennis toe bij de keuze van RxJS-schedulers.
---

# Basiskennis van taken en schedulers

## Wat is synchrone verwerking?
Synchrone verwerking wordt onmiddellijk uitgevoerd in de volgorde waarin de code is geschreven, en gaat niet verder naar de volgende verwerking totdat de vorige is voltooid.

#### Voorbeeld
```ts
console.log('A');
console.log('B');
console.log('C');

// Output:
// A
// B
// C
```


## Wat is asynchrone verwerking?
Asynchrone verwerking wordt niet onmiddellijk uitgevoerd, maar pas na het voltooien van de huidige synchrone verwerking.
Er zijn twee soorten asynchrone verwerking: "macrotasks" en "microtasks".


## Macrotasks
- Taken die in de volgende cyclus van de event loop worden uitgevoerd.
- Voorbeelden: `setTimeout`, `setInterval`, browserevents

#### Uitvoeringsvoorbeeld
```ts
console.log('Start');
setTimeout(() => console.log('Macro Task'), 0);
console.log('End');

// Output:
// Start
// End
// Macro Task
```

### Relatie met RxJS
- `asyncScheduler`
  - Gebruikt intern `setTimeout`
  - Werkt als macrotask

```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

of('Hello')
  .pipe(observeOn(asyncScheduler))
  .subscribe(console.log);

// Output:
// Hello
```


## Microtasks
- Taken die onmiddellijk na voltooiing van de huidige taak worden uitgevoerd, vóór het begin van de volgende taak.
- Voorbeelden: `Promise.then`, `queueMicrotask`

#### Uitvoeringsvoorbeeld
```ts
console.log('Start');
Promise.resolve().then(() => console.log('Micro Task'));
console.log('End');

// Output:
// Start
// End
// Micro Task
```

### Relatie met RxJS
- `asapScheduler`
  - Gebruikt intern `Promise.resolve().then()`
  - Werkt als microtask

```ts
import { of, asapScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

of('Hi')
  .pipe(observeOn(asapScheduler))
  .subscribe(console.log);

// Output:
// Hi
```


## Synchrone taken
- Normale code die onmiddellijk wordt uitgevoerd.

### Relatie met RxJS
- `queueScheduler`
  - Lijkt synchroon, maar maakt fijne controle mogelijk door task queuing.

```ts
import { of, queueScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

of('Now')
  .pipe(observeOn(queueScheduler))
  .subscribe(console.log);

// Output:
// Now
```


## Samenvatting van uitvoervolgorde

#### Codevoorbeeld
```ts
console.log('1');

setTimeout(() => console.log('2 (setTimeout)'), 0);
Promise.resolve().then(() => console.log('3 (Promise)'));

console.log('4');

// Output:
// 1
// 4
// 3 (Promise) 👈 Microtask
// 2 (setTimeout) 👈 Macrotask
```


## Overzichtstabel taken en RxJS-schedulers

| Type         | Voorbeeld                          | RxJS Scheduler  |
|--------------|------------------------------|---------------------|
| Synchrone verwerking     | Normale code                  | `queueScheduler`    |
| Microtask | Promise.then, queueMicrotask | `asapScheduler`     |
| Macrotask | setTimeout, setInterval      | `asyncScheduler`    |
