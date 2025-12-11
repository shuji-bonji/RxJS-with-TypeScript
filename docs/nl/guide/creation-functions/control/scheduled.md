---
description: Deze presentatie beschrijft in detail hoe u de scheduled() functie van RxJS gebruikt om een scheduler te specificeren, een Observable te genereren en de timing van uitvoering te controleren, met praktische codevoorbeelden.
---

# scheduled()

[📘 RxJS Officiële Documentatie - scheduled](https://rxjs.dev/api/index/function/scheduled)

`scheduled()` is een Creation Function waarmee u expliciet een scheduler kunt specificeren bij het genereren van Observables uit gegevensbronnen zoals arrays, Promises en Iterables. Dit maakt fijne controle mogelijk over de timing van uitvoering (synchroon of asynchroon) en is nuttig voor testen en optimalisatie van UI-prestaties.

## Basisgebruik

### Een eenvoudige array omzetten naar een Observable

```typescript
import { scheduled, asyncScheduler } from 'rxjs';

// Issue array asynchronously
const observable$ = scheduled([1, 2, 3], asyncScheduler);

console.log('Subscription started');
observable$.subscribe({
  next: val => console.log('Value:', val),
  complete: () => console.log('Complete')
});
console.log('Subscription ended');

// Output:
// Subscription started
// Subscription ended
// Value: 1
// Value: 2
// Value: 3
// Complete
```

> [!IMPORTANT]
> **Verschil tussen synchroon en asynchroon**
>
> Het gebruik van `asyncScheduler` maakt waarde-emissie asynchroon. Daarom is de uitvoervolgorde: "Subscription started" → "Subscription ended" → "Value: 1".

### Vergelijking met from()

```typescript
import { from, scheduled, asyncScheduler } from 'rxjs';

// from() - default is synchronous
console.log('=== from() ===');
from([1, 2, 3]).subscribe(val => console.log('Value:', val));
console.log('Subscription ended');

// Output:
// === from() ===
// Value: 1
// Value: 2
// Value: 3
// Subscription ended

// scheduled() - explicitly asynchronous
console.log('=== scheduled() ===');
scheduled([1, 2, 3], asyncScheduler).subscribe(val => console.log('Value:', val));
console.log('Subscription ended');

// Output:
// === scheduled() ===
// Subscription ended
// Value: 1
// Value: 2
// Value: 3
```

## Typen schedulers

RxJS biedt meerdere schedulers die voor verschillende doeleinden kunnen worden gebruikt.

| Scheduler | Uitvoeringstiming | Basistechnologie | Hoofdgebruik |
|-----------|------------------|------------------|--------------|
| `queueScheduler` | Synchroon (queue) | Directe uitvoering | Standaard, synchrone verwerking |
| `asyncScheduler` | Asynchroon | `setTimeout` | UI-optimalisatie, lange verwerking |
| `asapScheduler` | Snelste asynchroon | `Promise` (microtask) | Asynchrone verwerking met hoge prioriteit |
| `animationFrameScheduler` | Animatieframe | `requestAnimationFrame` | Animatie, UI-rendering |

### queueScheduler (synchrone uitvoering)

```typescript
import { scheduled, queueScheduler } from 'rxjs';

console.log('Start');
scheduled([1, 2, 3], queueScheduler).subscribe(val => console.log('Value:', val));
console.log('End');

// Output:
// Start
// Value: 1
// Value: 2
// Value: 3
// End
```

### asyncScheduler (asynchrone uitvoering)

```typescript
import { scheduled, asyncScheduler } from 'rxjs';

console.log('Start');
scheduled([1, 2, 3], asyncScheduler).subscribe(val => console.log('Value:', val));
console.log('End');

// Output:
// Start
// End
// Value: 1
// Value: 2
// Value: 3
```

### asapScheduler (microtask)

```typescript
import { scheduled, asapScheduler } from 'rxjs';

console.log('Start');
scheduled([1, 2, 3], asapScheduler).subscribe(val => console.log('Value:', val));
console.log('End');

// Output:
// Start
// End
// Value: 1
// Value: 2
// Value: 3
```

> [!TIP]
> **asyncScheduler vs asapScheduler**
>
> - `asyncScheduler`: Gebaseerd op `setTimeout` (macrotask)
> - `asapScheduler`: Gebaseerd op `Promise` (microtask)
>
> `asapScheduler` voert sneller uit, maar beide zijn asynchroon.

### animationFrameScheduler (animatie)

```typescript
import { scheduled, animationFrameScheduler, map } from 'rxjs';
// Update values at each animation frame
const positions = [0, 50, 100, 150, 200];
const animation$ = scheduled(positions, animationFrameScheduler).pipe(
  map(pos => `Position: ${pos}px`)
);

animation$.subscribe(position => {
  console.log(position);
  // Update DOM here
});

// Output: (at each animation frame)
// Position: 0px
// Position: 50px
// Position: 100px
// Position: 150px
// Position: 200px
```

## Praktische patronen

### Massale gegevensverwerking zonder de UI te blokkeren

```typescript
import { scheduled, asyncScheduler, map, bufferCount } from 'rxjs';
// Process 1 million data items
const largeArray = Array.from({ length: 1000000 }, (_, i) => i);

// ❌ Bad example: synchronous processing (UI will be blocked)
// from(largeArray).subscribe(processData);

// ✅ Good example: asynchronous processing (UI will not be blocked)
scheduled(largeArray, asyncScheduler).pipe(
  bufferCount(1000), // Batch process 1000 at a time
  map(batch => batch.reduce((sum, val) => sum + val, 0))
).subscribe({
  next: sum => console.log('Batch total:', sum),
  complete: () => console.log('Processing complete')
});

console.log('UI remains responsive');
```

### Combinatie met Promise

```typescript
import { scheduled, asyncScheduler, mergeMap } from 'rxjs';
interface User {
  id: number;
  name: string;
}

const userIds = [1, 2, 3, 4, 5];

// Fetch multiple users asynchronously
scheduled(userIds, asyncScheduler).pipe(
  mergeMap(id =>
    fetch(`https://api.example.com/users/${id}`).then(res => res.json())
  )
).subscribe({
  next: (user: User) => console.log('User:', user),
  error: error => console.error('Error:', error),
  complete: () => console.log('All users fetched')
});
```

### Genereren uit Iterable

```typescript
import { scheduled, asyncScheduler } from 'rxjs';

// Convert Set with scheduling
const uniqueNumbers = new Set([1, 2, 3, 4, 5]);
const observable$ = scheduled(uniqueNumbers, asyncScheduler);

observable$.subscribe({
  next: val => console.log('Value:', val),
  complete: () => console.log('Complete')
});

// Convert Map with scheduling
const userMap = new Map([
  [1, 'Alice'],
  [2, 'Bob'],
  [3, 'Charlie']
]);

scheduled(userMap, asyncScheduler).subscribe({
  next: ([id, name]) => console.log(`ID: ${id}, Name: ${name}`),
  complete: () => console.log('Complete')
});
```

## Gebruik bij testen

`scheduled()` kan worden gecombineerd met TestScheduler om tests te schrijven met tijdcontrole.

### Basistesten

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

  it('emits array elements in order', () => {
    testScheduler.run(({ expectObservable }) => {
      const source$ = scheduled([1, 2, 3], testScheduler);
      const expected = '(abc|)';
      const values = { a: 1, b: 2, c: 3 };

      expectObservable(source$).toBe(expected, values);
    });
  });
});
```

### Testen van asynchrone verwerking

```typescript
import { scheduled, asyncScheduler, delay } from 'rxjs';
import { TestScheduler } from 'rxjs/testing';

describe('Testing asynchronous processing', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('virtually tests delayed processing', () => {
    testScheduler.run(({ expectObservable }) => {
      const source$ = scheduled([1, 2, 3], testScheduler).pipe(
        delay(1000, testScheduler)
      );

      // Emit after 1000ms (virtual time)
      const expected = '1000ms (abc|)';
      const values = { a: 1, b: 2, c: 3 };

      expectObservable(source$).toBe(expected, values);
    });
  });
});
```

> [!TIP]
> **Voordelen van TestScheduler**
>
> - Testen zonder daadwerkelijk te wachten op tijd
> - Asynchrone verwerking synchroon testen
> - Testtijd drastisch verkorten

## Veelvoorkomende gebruiksvoorbeelden

### 1. Gepagineerd ophalen van gegevens

```typescript
import { scheduled, asyncScheduler, mergeMap, toArray } from 'rxjs';
interface Page {
  page: number;
  data: any[];
}

// List of page numbers
const pages = [1, 2, 3, 4, 5];

// Fetch each page asynchronously
const allData$ = scheduled(pages, asyncScheduler).pipe(
  mergeMap(page =>
    fetch(`https://api.example.com/items?page=${page}`)
      .then(res => res.json())
  ),
  toArray() // Combine all pages of data
);

allData$.subscribe({
  next: data => console.log('All data:', data),
  complete: () => console.log('Fetch complete')
});
```

### 2. Batchverwerking

```typescript
import { scheduled, asyncScheduler, bufferCount, mergeMap, delay } from 'rxjs';
// Process large number of tasks 1000 at a time
const tasks = Array.from({ length: 10000 }, (_, i) => `Task-${i}`);

scheduled(tasks, asyncScheduler).pipe(
  bufferCount(1000), // Batch 1000 at a time
  mergeMap(batch => {
    console.log(`Processing batch: ${batch.length} items`);
    // Execute batch processing
    return processBatch(batch);
  })
).subscribe({
  complete: () => console.log('All batch processing complete')
});

function processBatch(batch: string[]): Promise<void> {
  // Batch processing logic
  return Promise.resolve();
}
```

### 3. Animatie-implementatie

```typescript
import { scheduled, animationFrameScheduler, map } from 'rxjs';
// Generate values from 0 to 100
const frames = Array.from({ length: 100 }, (_, i) => i);

// Execute at each animation frame
const animation$ = scheduled(frames, animationFrameScheduler).pipe(
  map(frame => ({
    progress: frame / 100,
    position: frame * 5 // Move from 0px to 500px
  }))
);

animation$.subscribe({
  next: ({ progress, position }) => {
    const element = document.getElementById('animated-box');
    if (element) {
      element.style.transform = `translateX(${position}px)`;
      console.log(`Progress: ${(progress * 100).toFixed(0)}%`);
    }
  },
  complete: () => console.log('Animation complete')
});
```

### 4. Taakverwerking met prioriteit

```typescript
import { scheduled, asapScheduler, asyncScheduler } from 'rxjs';

// High priority tasks (asapScheduler = microtask)
const highPriorityTasks = ['Urgent task 1', 'Urgent task 2'];
const highPriority$ = scheduled(highPriorityTasks, asapScheduler);

// Low priority tasks (asyncScheduler = macrotask)
const lowPriorityTasks = ['Normal task 1', 'Normal task 2'];
const lowPriority$ = scheduled(lowPriorityTasks, asyncScheduler);

console.log('Task start');

highPriority$.subscribe(task => console.log('High priority:', task));
lowPriority$.subscribe(task => console.log('Low priority:', task));

console.log('Task registration complete');

// Output:
// Task start
// Task registration complete
// High priority: Urgent task 1
// High priority: Urgent task 2
// Low priority: Normal task 1
// Low priority: Normal task 2
```

## scheduled() opties

`scheduled()` heeft de volgende handtekening.

```typescript
function scheduled<T>(
  input: ObservableInput<T>,
  scheduler: SchedulerLike
): Observable<T>
```

### Ondersteunde invoertypen

- **Array**: `T[]`
- **Promise**: `Promise<T>`
- **Iterable**: `Iterable<T>` (Set, Map, Generator, etc.)
- **Observable**: `Observable<T>`
- **ArrayLike**: `ArrayLike<T>`

```typescript
import { scheduled, asyncScheduler } from 'rxjs';

// Array
scheduled([1, 2, 3], asyncScheduler);

// Promise
scheduled(Promise.resolve('result'), asyncScheduler);

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

## Veelvoorkomende fouten en hun oplossingen

### 1. Vergeten om scheduler te specificeren

**Foutvoorbeeld:**
```typescript
// ❌ Error: 2nd argument required
const observable$ = scheduled([1, 2, 3]);
```

**Oplossing:**
```typescript
// ✅ Correct: specify scheduler
const observable$ = scheduled([1, 2, 3], asyncScheduler);
```

### 2. animationFrameScheduler gebruiken in een browseromgeving

**Probleem:**
In Node.js-omgevingen bestaat `requestAnimationFrame` niet, wat fouten veroorzaakt.

**Oplossing:**
```typescript
import { scheduled, animationFrameScheduler, asyncScheduler } from 'rxjs';

// Check if browser environment
const scheduler = typeof window !== 'undefined'
  ? animationFrameScheduler
  : asyncScheduler;

const observable$ = scheduled([1, 2, 3], scheduler);
```

### 3. Verwarring tussen synchrone en asynchrone verwerking

**Probleem:**
```typescript
// Expecting asynchronous execution, but actually synchronous
scheduled([1, 2, 3], queueScheduler).subscribe(val => {
  console.log(val);
});
console.log('Complete'); // ← 1, 2, 3 are output before this
```

**Oplossing:**
```typescript
// Explicitly specify asynchronous
scheduled([1, 2, 3], asyncScheduler).subscribe(val => {
  console.log(val);
});
console.log('Complete'); // ← 1, 2, 3 are output after this
```

## Vergelijking met from()

| Functie | from() | scheduled() |
|---------|--------|-------------|
| Scheduler-specificatie | ❌ Niet mogelijk (alleen standaard) | ✅ Expliciet specificeerbaar |
| Synchrone/asynchrone controle | ❌ Kan niet controleren | ✅ Controleerbaar |
| Testgemak | Normaal | ✅ Tijd controleerbaar met TestScheduler |
| Eenvoud | ✅ Eenvoudig | Enigszins complex |
| Gebruiksscenario | Basisconversie | Wanneer controle over uitvoeringstiming nodig is |

> [!TIP]
> **Punten om te overwegen bij het kiezen**
>
> - **Gebruik in principe `from()`**: Wanneer scheduler-controle niet nodig is
> - **Gebruik `scheduled()` wanneer**:
>   - U UI-blokkering wilt voorkomen
>   - U tijdcontrole nodig hebt in tests
>   - Animatie-implementatie
>   - Taakverwerking met prioriteit

## Best Practices

### 1. Gebruik asyncScheduler voor verwerking van grote gegevens

```typescript
// ✅ Good example: doesn't block UI
scheduled(largeArray, asyncScheduler).pipe(
  map(processHeavyTask)
).subscribe();
```

### 2. Gebruik TestScheduler voor testen

```typescript
// ✅ Good example: control time virtually
testScheduler.run(({ expectObservable }) => {
  const source$ = scheduled([1, 2, 3], testScheduler);
  expectObservable(source$).toBe('(abc|)', { a: 1, b: 2, c: 3 });
});
```

### 3. Gebruik animationFrameScheduler voor animatie

```typescript
// ✅ Good example: match browser repaint timing
scheduled(frames, animationFrameScheduler).subscribe(updateUI);
```

### 4. Selecteer een scheduler die het beste bij uw omgeving past

```typescript
// ✅ Good example: switch according to environment
const scheduler = process.env.NODE_ENV === 'test'
  ? queueScheduler
  : asyncScheduler;

const source$ = scheduled(data, scheduler);
```

## Samenvatting

`scheduled()` is een Creation Function die een Observable creëert door expliciet een scheduler te specificeren.

**Belangrijkste kenmerken:**
- Expliciete controle over uitvoeringstiming (synchroon of asynchroon)
- Meerdere schedulers om uit te kiezen
- Gemakkelijk te testen met TestScheduler
- Effectief voor het voorkomen van UI-blokkering

**Gebruiksscenario's:**
- Asynchrone verwerking van grote hoeveelheden gegevens
- Implementatie van animaties
- Tijdcontrole bij testen
- Taakverwerking met prioriteit

**Opmerkingen:**
- Specificeer altijd een scheduler
- Selecteer de juiste scheduler voor uw omgeving
- Begrijp het verschil tussen from() en scheduled()

**Aanbevolen gebruik:**
- UI-optimalisatie: `asyncScheduler`
- Animatie: `animationFrameScheduler`
- Testen: `TestScheduler`
- Hoge prioriteit: `asapScheduler`

## Gerelateerde pagina's

- [using()](/nl/guide/creation-functions/control/using) - Observable met resource-controle
- [Control Creation Functions](/nl/guide/creation-functions/control/) - Vergelijking van scheduled() en using()
- [Scheduler Types](/nl/guide/schedulers/types) - Details van schedulers
- [from()](/nl/guide/creation-functions/basic/from) - Basis Observable-generatie

## Referenties

- [RxJS Officiële Documentatie - scheduled](https://rxjs.dev/api/index/function/scheduled)
- [RxJS Officiële Documentatie - Scheduler](https://rxjs.dev/guide/scheduler)
- [RxJS Officiële Documentatie - TestScheduler](https://rxjs.dev/api/testing/TestScheduler)
