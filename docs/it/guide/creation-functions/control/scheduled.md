---
description: Questa presentazione descrive in dettaglio come usare la funzione scheduled() di RxJS per specificare uno scheduler, generare un Observable e controllare il timing di esecuzione, con esempi di codice pratici.
---

# scheduled()

[📘 Documentazione Ufficiale RxJS - scheduled](https://rxjs.dev/api/index/function/scheduled)

`scheduled()` è una Funzione di Creazione che ti permette di specificare esplicitamente uno scheduler quando generi Observable da sorgenti dati come array, Promise e Iterable. Questo permette un controllo fine del timing di esecuzione (sincrono o asincrono) ed è utile per testing e ottimizzazione delle performance UI.

## Uso Base

### Convertire un semplice array in Observable

```typescript
import { scheduled, asyncScheduler } from 'rxjs';

// Emetti array asincronamente
const observable$ = scheduled([1, 2, 3], asyncScheduler);

console.log('Subscription iniziata');
observable$.subscribe({
  next: val => console.log('Valore:', val),
  complete: () => console.log('Completo')
});
console.log('Subscription terminata');

// Output:
// Subscription iniziata
// Subscription terminata
// Valore: 1
// Valore: 2
// Valore: 3
// Completo
```

> [!IMPORTANT]
> **Differenza tra Sincrono e Asincrono**
>
> Usare `asyncScheduler` rende l'emissione dei valori asincrona. Quindi, l'ordine di output è: "Subscription iniziata" → "Subscription terminata" → "Valore: 1".

### Confronto con from()

```typescript
import { from, scheduled, asyncScheduler } from 'rxjs';

// from() - default è sincrono
console.log('=== from() ===');
from([1, 2, 3]).subscribe(val => console.log('Valore:', val));
console.log('Subscription terminata');

// Output:
// === from() ===
// Valore: 1
// Valore: 2
// Valore: 3
// Subscription terminata

// scheduled() - esplicitamente asincrono
console.log('=== scheduled() ===');
scheduled([1, 2, 3], asyncScheduler).subscribe(val => console.log('Valore:', val));
console.log('Subscription terminata');

// Output:
// === scheduled() ===
// Subscription terminata
// Valore: 1
// Valore: 2
// Valore: 3
```

## Tipi di Scheduler

RxJS fornisce più scheduler, che possono essere usati per scopi diversi.

| Scheduler | Timing Esecuzione | Tecnologia Base | Uso Principale |
|-----------|-----------------|----------------|----------|
| `queueScheduler` | Sincrono (coda) | Esecuzione immediata | Default, elaborazione sincrona |
| `asyncScheduler` | Asincrono | `setTimeout` | Ottimizzazione UI, elaborazione lunga |
| `asapScheduler` | Asincrono più veloce | `Promise` (microtask) | Elaborazione asincrona ad alta priorità |
| `animationFrameScheduler` | Frame di animazione | `requestAnimationFrame` | Animazione, rendering UI |

### queueScheduler (esecuzione sincrona)

```typescript
import { scheduled, queueScheduler } from 'rxjs';

console.log('Inizio');
scheduled([1, 2, 3], queueScheduler).subscribe(val => console.log('Valore:', val));
console.log('Fine');

// Output:
// Inizio
// Valore: 1
// Valore: 2
// Valore: 3
// Fine
```

### asyncScheduler (esecuzione asincrona)

```typescript
import { scheduled, asyncScheduler } from 'rxjs';

console.log('Inizio');
scheduled([1, 2, 3], asyncScheduler).subscribe(val => console.log('Valore:', val));
console.log('Fine');

// Output:
// Inizio
// Fine
// Valore: 1
// Valore: 2
// Valore: 3
```

### asapScheduler (microtask)

```typescript
import { scheduled, asapScheduler } from 'rxjs';

console.log('Inizio');
scheduled([1, 2, 3], asapScheduler).subscribe(val => console.log('Valore:', val));
console.log('Fine');

// Output:
// Inizio
// Fine
// Valore: 1
// Valore: 2
// Valore: 3
```

> [!TIP]
> **asyncScheduler vs asapScheduler**
>
> - `asyncScheduler`: basato su `setTimeout` (macrotask)
> - `asapScheduler`: basato su `Promise` (microtask)
>
> `asapScheduler` esegue più velocemente, ma entrambi sono asincroni.

### animationFrameScheduler (animazione)

```typescript
import { scheduled, animationFrameScheduler, map } from 'rxjs';
// Aggiorna valori ad ogni frame di animazione
const positions = [0, 50, 100, 150, 200];
const animation$ = scheduled(positions, animationFrameScheduler).pipe(
  map(pos => `Posizione: ${pos}px`)
);

animation$.subscribe(position => {
  console.log(position);
  // Aggiorna DOM qui
});

// Output: (ad ogni frame di animazione)
// Posizione: 0px
// Posizione: 50px
// Posizione: 100px
// Posizione: 150px
// Posizione: 200px
```

## Pattern Pratici

### Elaborazione grandi quantità di dati senza bloccare la UI

```typescript
import { scheduled, asyncScheduler, map, bufferCount } from 'rxjs';
// Elabora 1 milione di elementi
const largeArray = Array.from({ length: 1000000 }, (_, i) => i);

// ❌ Cattivo esempio: elaborazione sincrona (UI si blocca)
// from(largeArray).subscribe(processData);

// ✅ Buon esempio: elaborazione asincrona (UI non si blocca)
scheduled(largeArray, asyncScheduler).pipe(
  bufferCount(1000), // Elabora in batch di 1000 alla volta
  map(batch => batch.reduce((sum, val) => sum + val, 0))
).subscribe({
  next: sum => console.log('Totale batch:', sum),
  complete: () => console.log('Elaborazione completata')
});

console.log('La UI rimane reattiva');
```

### Combinazione con Promise

```typescript
import { scheduled, asyncScheduler, mergeMap } from 'rxjs';
interface User {
  id: number;
  name: string;
}

const userIds = [1, 2, 3, 4, 5];

// Recupera più utenti asincronamente
scheduled(userIds, asyncScheduler).pipe(
  mergeMap(id =>
    fetch(`https://api.example.com/users/${id}`).then(res => res.json())
  )
).subscribe({
  next: (user: User) => console.log('Utente:', user),
  error: error => console.error('Errore:', error),
  complete: () => console.log('Tutti gli utenti recuperati')
});
```

### Generazione da Iterable

```typescript
import { scheduled, asyncScheduler } from 'rxjs';

// Converti Set con scheduling
const uniqueNumbers = new Set([1, 2, 3, 4, 5]);
const observable$ = scheduled(uniqueNumbers, asyncScheduler);

observable$.subscribe({
  next: val => console.log('Valore:', val),
  complete: () => console.log('Completo')
});

// Converti Map con scheduling
const userMap = new Map([
  [1, 'Alice'],
  [2, 'Bob'],
  [3, 'Charlie']
]);

scheduled(userMap, asyncScheduler).subscribe({
  next: ([id, name]) => console.log(`ID: ${id}, Nome: ${name}`),
  complete: () => console.log('Completo')
});
```

## Uso nel Testing

`scheduled()` può essere combinato con TestScheduler per scrivere test con controllo del tempo.

### Testing base

```typescript
import { TestScheduler } from 'rxjs/testing';
import { scheduled } from 'rxjs';

describe('scheduled()', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('emette elementi array in ordine', () => {
    testScheduler.run(({ expectObservable }) => {
      const source$ = scheduled([1, 2, 3], testScheduler);
      const expected = '(abc|)';
      const values = { a: 1, b: 2, c: 3 };

      expectObservable(source$).toBe(expected, values);
    });
  });
});
```

### Testing elaborazione asincrona

```typescript
import { scheduled, asyncScheduler, delay } from 'rxjs';
import { TestScheduler } from 'rxjs/testing';

describe('Testing elaborazione asincrona', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('testa virtualmente elaborazione ritardata', () => {
    testScheduler.run(({ expectObservable }) => {
      const source$ = scheduled([1, 2, 3], testScheduler).pipe(
        delay(1000, testScheduler)
      );

      // Emetti dopo 1000ms (tempo virtuale)
      const expected = '1000ms (abc|)';
      const values = { a: 1, b: 2, c: 3 };

      expectObservable(source$).toBe(expected, values);
    });
  });
});
```

> [!TIP]
> **Vantaggi di TestScheduler**
>
> - Testa senza aspettare realmente il tempo
> - Testa elaborazione asincrona sincronamente
> - Riduce drasticamente il tempo di esecuzione dei test

## Esempi di Utilizzo Comuni

### 1. Recupero dati paginati

```typescript
import { scheduled, asyncScheduler, mergeMap, toArray } from 'rxjs';
interface Page {
  page: number;
  data: any[];
}

// Lista dei numeri di pagina
const pages = [1, 2, 3, 4, 5];

// Recupera ogni pagina asincronamente
const allData$ = scheduled(pages, asyncScheduler).pipe(
  mergeMap(page =>
    fetch(`https://api.example.com/items?page=${page}`)
      .then(res => res.json())
  ),
  toArray() // Combina tutti i dati delle pagine
);

allData$.subscribe({
  next: data => console.log('Tutti i dati:', data),
  complete: () => console.log('Recupero completato')
});
```

### 2. Elaborazione batch

```typescript
import { scheduled, asyncScheduler, bufferCount, mergeMap, delay } from 'rxjs';
// Elabora gran numero di task 1000 alla volta
const tasks = Array.from({ length: 10000 }, (_, i) => `Task-${i}`);

scheduled(tasks, asyncScheduler).pipe(
  bufferCount(1000), // Batch di 1000 alla volta
  mergeMap(batch => {
    console.log(`Elaborazione batch: ${batch.length} elementi`);
    // Esegui elaborazione batch
    return processBatch(batch);
  })
).subscribe({
  complete: () => console.log('Tutta l\'elaborazione batch completata')
});

function processBatch(batch: string[]): Promise<void> {
  // Logica elaborazione batch
  return Promise.resolve();
}
```

### 3. Implementazione animazione

```typescript
import { scheduled, animationFrameScheduler, map } from 'rxjs';
// Genera valori da 0 a 100
const frames = Array.from({ length: 100 }, (_, i) => i);

// Esegui ad ogni frame di animazione
const animation$ = scheduled(frames, animationFrameScheduler).pipe(
  map(frame => ({
    progress: frame / 100,
    position: frame * 5 // Muovi da 0px a 500px
  }))
);

animation$.subscribe({
  next: ({ progress, position }) => {
    const element = document.getElementById('animated-box');
    if (element) {
      element.style.transform = `translateX(${position}px)`;
      console.log(`Progresso: ${(progress * 100).toFixed(0)}%`);
    }
  },
  complete: () => console.log('Animazione completata')
});
```

### 4. Elaborazione task prioritizzata

```typescript
import { scheduled, asapScheduler, asyncScheduler } from 'rxjs';

// Task ad alta priorità (asapScheduler = microtask)
const highPriorityTasks = ['Task urgente 1', 'Task urgente 2'];
const highPriority$ = scheduled(highPriorityTasks, asapScheduler);

// Task a bassa priorità (asyncScheduler = macrotask)
const lowPriorityTasks = ['Task normale 1', 'Task normale 2'];
const lowPriority$ = scheduled(lowPriorityTasks, asyncScheduler);

console.log('Inizio task');

highPriority$.subscribe(task => console.log('Alta priorità:', task));
lowPriority$.subscribe(task => console.log('Bassa priorità:', task));

console.log('Registrazione task completata');

// Output:
// Inizio task
// Registrazione task completata
// Alta priorità: Task urgente 1
// Alta priorità: Task urgente 2
// Bassa priorità: Task normale 1
// Bassa priorità: Task normale 2
```

## Opzioni di scheduled()

`scheduled()` ha la seguente firma.

```typescript
function scheduled<T>(
  input: ObservableInput<T>,
  scheduler: SchedulerLike
): Observable<T>
```

### Tipi di input supportati

- **Array**: `T[]`
- **Promise**: `Promise<T>`
- **Iterable**: `Iterable<T>` (Set, Map, Generator, ecc.)
- **Observable**: `Observable<T>`
- **ArrayLike**: `ArrayLike<T>`

```typescript
import { scheduled, asyncScheduler } from 'rxjs';

// Array
scheduled([1, 2, 3], asyncScheduler);

// Promise
scheduled(Promise.resolve('risultato'), asyncScheduler);

// Set
scheduled(new Set([1, 2, 3]), asyncScheduler);

// Generator
function* generator() {
  yield 1;
  yield 2;
  yield 3;
}
scheduled(generator(), asyncScheduler);
```

## Errori Comuni e Loro Soluzioni

### 1. Dimenticato di specificare lo scheduler

**Esempio di errore:**
```typescript
// ❌ Errore: secondo argomento richiesto
const observable$ = scheduled([1, 2, 3]);
```

**Soluzione:**
```typescript
// ✅ Corretto: specifica lo scheduler
const observable$ = scheduled([1, 2, 3], asyncScheduler);
```

### 2. Uso di animationFrameScheduler in ambiente browser

**Problema:**
In ambienti Node.js, `requestAnimationFrame` non esiste, causando errori.

**Soluzione:**
```typescript
import { scheduled, animationFrameScheduler, asyncScheduler } from 'rxjs';

// Verifica se ambiente browser
const scheduler = typeof window !== 'undefined'
  ? animationFrameScheduler
  : asyncScheduler;

const observable$ = scheduled([1, 2, 3], scheduler);
```

### 3. Confusione tra elaborazione sincrona e asincrona

**Problema:**
```typescript
// Aspettandosi esecuzione asincrona, ma in realtà sincrona
scheduled([1, 2, 3], queueScheduler).subscribe(val => {
  console.log(val);
});
console.log('Completo'); // ← 1, 2, 3 vengono emessi prima di questo
```

**Soluzione:**
```typescript
// Specifica esplicitamente asincrono
scheduled([1, 2, 3], asyncScheduler).subscribe(val => {
  console.log(val);
});
console.log('Completo'); // ← 1, 2, 3 vengono emessi dopo questo
```

## Confronto con from()

| Caratteristica | from() | scheduled() |
|---------|--------|-------------|
| Specifica scheduler | ❌ Non possibile (solo default) | ✅ Specificabile esplicitamente |
| Controllo sincrono/asincrono | ❌ Non controllabile | ✅ Controllabile |
| Facilità testing | Normale | ✅ Tempo controllabile con TestScheduler |
| Semplicità | ✅ Semplice | Un po' complessa |
| Caso d'uso | Conversione base | Quando serve controllo timing esecuzione |

> [!TIP]
> **Punti per la scelta**
>
> - **Usa `from()` di base**: Quando non serve controllo scheduler
> - **Usa `scheduled()` quando**:
>   - Vuoi evitare blocco UI
>   - Serve controllo tempo nei test
>   - Implementazione animazioni
>   - Elaborazione task prioritizzata

## Best Practice

### 1. Usa asyncScheduler per elaborazione grandi dati

```typescript
// ✅ Buon esempio: non blocca UI
scheduled(largeArray, asyncScheduler).pipe(
  map(processHeavyTask)
).subscribe();
```

### 2. Usa TestScheduler per il testing

```typescript
// ✅ Buon esempio: controlla tempo virtualmente
testScheduler.run(({ expectObservable }) => {
  const source$ = scheduled([1, 2, 3], testScheduler);
  expectObservable(source$).toBe('(abc|)', { a: 1, b: 2, c: 3 });
});
```

### 3. Usa animationFrameScheduler per le animazioni

```typescript
// ✅ Buon esempio: sincronizza con timing repaint browser
scheduled(frames, animationFrameScheduler).subscribe(updateUI);
```

### 4. Seleziona lo scheduler più adatto al tuo ambiente

```typescript
// ✅ Buon esempio: cambia secondo l'ambiente
const scheduler = process.env.NODE_ENV === 'test'
  ? queueScheduler
  : asyncScheduler;

const source$ = scheduled(data, scheduler);
```

## Riepilogo

`scheduled()` è una Funzione di Creazione che crea un Observable specificando esplicitamente uno scheduler.

**Caratteristiche Principali:**
- Controllo esplicito del timing di esecuzione (sincrono o asincrono)
- Più scheduler tra cui scegliere
- Facile da testare con TestScheduler
- Efficace per evitare blocco UI

**Scenari di utilizzo:**
- Elaborazione asincrona di grandi quantità di dati
- Implementazione di animazioni
- Controllo del tempo nel testing
- Elaborazione task prioritizzata

**Note:**
- Specifica sempre uno scheduler
- Seleziona lo scheduler appropriato per il tuo ambiente
- Comprendi la differenza tra from() e scheduled()

**Uso raccomandato:**
- Ottimizzazione UI: `asyncScheduler`
- Animazione: `animationFrameScheduler`
- Testing: `TestScheduler`
- Alta priorità: `asapScheduler`

## Pagine Correlate

- [using()](/it/guide/creation-functions/control/using) - Observable con controllo delle risorse
- [Funzioni di Creazione di Controllo](/it/guide/creation-functions/control/) - Confronto tra scheduled() e using()
- [Tipi di Scheduler](/it/guide/schedulers/types) - Dettagli sugli scheduler
- [from()](/it/guide/creation-functions/basic/from) - Generazione base di Observable

## Riferimenti

- [Documentazione Ufficiale RxJS - scheduled](https://rxjs.dev/api/index/function/scheduled)
- [Documentazione Ufficiale RxJS - Scheduler](https://rxjs.dev/guide/scheduler)
- [Documentazione Ufficiale RxJS - TestScheduler](https://rxjs.dev/api/testing/TestScheduler)
