---
description: Spiega dalle basi la classificazione dei task (sincroni, micro task, macro task) e la corrispondenza con ciascuno scheduler di RxJS. Comprendi il meccanismo dell'event loop di JavaScript, le differenze nell'ordine di esecuzione, l'implementazione e il comportamento di setTimeout, Promise, queueMicrotask, ecc., e acquisisci le conoscenze per applicarle alla scelta degli scheduler di RxJS.
---

# Concetti Base di Task e Scheduler

## Cosa Sono le Elaborazioni Sincrone
Le elaborazioni sincrone vengono eseguite immediatamente nell'ordine in cui il codice è scritto e non passano all'elaborazione successiva finché non termina quella precedente.

#### Esempio
```ts
console.log('A');
console.log('B');
console.log('C');

// Output:
// A
// B
// C
```


## Cosa Sono le Elaborazioni Asincrone
Le elaborazioni asincrone non vengono eseguite immediatamente, ma vengono eseguite dopo il completamento dell'elaborazione sincrona corrente.
Nelle elaborazioni asincrone esistono "macro task" e "micro task".


## Macro Task
- Task eseguiti nel ciclo successivo dell'event loop.
- Esempi: `setTimeout`, `setInterval`, eventi del browser

#### Esempio di Esecuzione
```ts
console.log('Start');
setTimeout(() => console.log('Macro Task'), 0);
console.log('End');

// Output:
// Start
// End
// Micro Task
```

### Corrispondenza in RxJS
- `asyncScheduler`
  - Utilizza `setTimeout` internamente
  - Opera come macro task

```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

of('Hello')
  .pipe(observeOn(asyncScheduler))
  .subscribe(console.log);

// Output:
// Hello
```


## Micro Task
- Task eseguiti immediatamente dopo il termine del task corrente, prima dell'inizio del task successivo.
- Esempi: `Promise.then`, `queueMicrotask`

#### Esempio di Esecuzione
```ts
console.log('Start');
Promise.resolve().then(() => console.log('Micro Task'));
console.log('End');

// Output:
// Start
// End
// Micro Task
```

### Corrispondenza in RxJS
- `asapScheduler`
  - Utilizza `Promise.resolve().then()` internamente
  - Opera come micro task

```ts
import { of, asapScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

of('Hi')
  .pipe(observeOn(asapScheduler))
  .subscribe(console.log);

// Output:
// Hi
```


## Task Sincroni
- Codice normale che viene eseguito immediatamente.

### Corrispondenza in RxJS
- `queueScheduler`
  - Appare sincrono, ma permette un controllo fine tramite l'accodamento dei task.

```ts
import { of, queueScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

of('Now')
  .pipe(observeOn(queueScheduler))
  .subscribe(console.log);

// Output:
// Now
```


## Riepilogo dell'Ordine di Esecuzione

#### Esempio di Codice
```ts
console.log('1');

setTimeout(() => console.log('2 (setTimeout)'), 0);
Promise.resolve().then(() => console.log('3 (Promise)'));

console.log('4');

// Output:
// 1
// 4
// 3 (Promise) 👈 Micro task
// 2 (setTimeout) 👈 Macro task
```


## Tabella di Corrispondenza Task e Scheduler RxJS

| Tipo         | Esempio                          | Scheduler RxJS      |
|--------------|----------------------------------|---------------------|
| Elaborazione sincrona     | Codice normale                  | `queueScheduler`    |
| Micro task | Promise.then, queueMicrotask | `asapScheduler`     |
| Macro task | setTimeout, setInterval      | `asyncScheduler`    |
