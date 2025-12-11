---
description: "generate() - Generieke lusgeneratie met flexibele voorwaardencontrole: Declaratieve while-achtige lussen voor Fibonacci, paginering en aangepast statusbeheer"
---

# generate() - Generieke lusgeneratie

`generate()` is een Creation Function die flexibele lusverwerking als Observable biedt door het specificeren van initiële status, voortzettingsvoorwaarde, statusupdate en resultaatselectie.

## Overzicht

`generate()` kan flexibele lusverwerking zoals while- en for-statements declaratief beschrijven. Het wordt gebruikt wanneer complexere voorwaarden of statusbeheer nodig zijn dan `range()`.

**Handtekening**:
```typescript
function generate<T, S>(
  initialState: S,
  condition: (state: S) => boolean,
  iterate: (state: S) => S,
  resultSelector?: (state: S) => T,
  scheduler?: SchedulerLike
): Observable<T>
```

**Parameters**:
- `initialState`: De initiële status van de lus
- `condition`: Functie om de voortzettingsvoorwaarde te bepalen (`false` beëindigt de lus)
- `iterate`: Functie om de status naar de volgende te brengen (statusupdate)
- `resultSelector`: Functie om een waarde te selecteren om uit te geven vanuit de status (indien weggelaten, wordt de status zelf uitgegeven)
- `scheduler`: Scheduler die waarden uitgeeft (weggelaten: geeft waarden synchroon uit)

**Officiële Documentatie**: [📘 RxJS Officieel: generate()](https://rxjs.dev/api/index/function/generate)

## Basisgebruik

### Patroon 1: Eenvoudige teller

Dit is het meest basale gebruik.

```typescript
import { generate } from 'rxjs';

// Tel van 1 tot 5
generate(
  1,              // Initiële status
  x => x <= 5,    // Voortzettingsvoorwaarde
  x => x + 1      // Statusupdate
).subscribe({
  next: value => console.log('Waarde:', value),
  complete: () => console.log('Voltooid')
});

// Output:
// Waarde: 1
// Waarde: 2
// Waarde: 3
// Waarde: 4
// Waarde: 5
// Voltooid
```

Deze code is equivalent aan de volgende while-statement:

```typescript
let x = 1;
while (x <= 5) {
  console.log('Waarde:', x);
  x = x + 1;
}
console.log('Voltooid');
```

### Patroon 2: Waarden converteren met resultSelector

U kunt de status scheiden van de uit te geven waarde.

```typescript
import { generate } from 'rxjs';

// Interne status is een teller, maar uitgegeven waarde is een gekwadrateerde waarde
generate(
  1,              // Initiële status: 1
  x => x <= 5,    // Voortzettingsvoorwaarde: x <= 5
  x => x + 1,     // Statusupdate: x + 1
  x => x * x      // Resultaatselectie: x^2 uitgeven
).subscribe(console.log);

// Output: 1, 4, 9, 16, 25
```

### Patroon 3: Complex statusobject

Complexe objecten kunnen als statussen worden gebruikt.

```typescript
import { generate } from 'rxjs';

interface State {
  count: number;
  sum: number;
}

// Cumulatieve som berekenen
generate<number, State>(
  { count: 1, sum: 0 },           // Initiële status
  state => state.count <= 5,      // Voortzettingsvoorwaarde
  state => ({                     // Statusupdate
    count: state.count + 1,
    sum: state.sum + state.count
  }),
  state => state.sum              // Resultaatselectie
).subscribe(console.log);

// Output: 0, 1, 3, 6, 10
// (0, 0+1, 0+1+2, 0+1+2+3, 0+1+2+3+4)
```

## Belangrijke kenmerken

### 1. While-statement-achtig gedrag

`generate()` biedt flexibele controle zoals een while-statement.

```typescript
import { generate } from "rxjs";

// While-statement
let i = 1;
while (i <= 10) {
  console.log(i);
  i = i * 2;
}

// Hetzelfde met generate()
generate(
  1,              // let i = 1;
  i => i <= 10,   // while (i <= 10)
  i => i * 2      // i = i * 2;
).subscribe(console.log);

// Output: 1, 2, 4, 8
```

### 2. Synchrone emissie

Standaard worden alle waarden **synchroon** uitgegeven bij abonnement.

```typescript
import { generate } from 'rxjs';

console.log('Voor abonnement');

generate(1, x => x <= 3, x => x + 1).subscribe(val => console.log('Waarde:', val));

console.log('Na abonnement');

// Output:
// Voor abonnement
// Waarde: 1
// Waarde: 2
// Waarde: 3
// Na abonnement
```

### 3. Pas op voor oneindige lussen

Als de voorwaarde altijd `true` is, krijgt u een oneindige lus.

```typescript
import { generate, take } from 'rxjs';
// ❌ Gevaar: oneindige lus (browser bevriest)
// generate(0, x => true, x => x + 1).subscribe(console.log);

// ✅ Veilig: gebruik take() om aantal te beperken
generate(
  0,
  x => true,  // Altijd true
  x => x + 1
).pipe(
  take(10)    // Alleen de eerste 10 ophalen
).subscribe(console.log);

// Output: 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
```

> [!WARNING]
> **Pas op voor oneindige lussen**:
> - Als de voorwaarde altijd `true` is, treedt een oneindige lus op
> - Gebruik `take()`, `takeWhile()` of `takeUntil()` om het aantal uitgaven te beperken
> - Of stel geschikte exit-voorwaarden in met voorwaardelijke functies

## Praktische use cases

### 1. Fibonacci-reeks

Voorbeeld van complexe statusovergangen.

```typescript
import { generate, take } from 'rxjs';
interface FibState {
  current: number;
  next: number;
}

// Eerste 10 termen van de Fibonacci-reeks
generate<number, FibState>(
  { current: 0, next: 1 },           // Initiële status: F(0)=0, F(1)=1
  state => true,                     // Oneindig gegenereerd
  state => ({                        // Statusupdate
    current: state.next,
    next: state.current + state.next
  }),
  state => state.current             // Huidige waarde uitgeven
).pipe(
  take(10)                           // Eerste 10 termen
).subscribe(console.log);

// Output: 0, 1, 1, 2, 3, 5, 8, 13, 21, 34
```

### 2. Exponentiële backoff

Dit is de exponentiële wachttijdgeneratie die wordt gebruikt in het retry-proces.

```typescript
import { generate } from 'rxjs';

interface RetryState {
  attempt: number;
  delay: number;
}

// Genereer vertraging voor exponentiële backoff (1, 2, 4, 8, 16 seconden)
generate<number, RetryState>(
  { attempt: 0, delay: 1000 },       // Initiële status: 1 seconde
  state => state.attempt < 5,        // Maximaal 5 pogingen
  state => ({                        // Statusupdate
    attempt: state.attempt + 1,
    delay: state.delay * 2           // Verdubbel de vertragingstijd
  }),
  state => state.delay               // Vertragingstijd uitgeven
).subscribe(delay => {
  console.log(`Retry ${delay / 1000} seconden later`);
});

// Output:
// Retry 1 seconde later
// Retry 2 seconden later
// Retry 4 seconden later
// Retry 8 seconden later
// Retry 16 seconden later
```

### 3. Pagineringscontrole

Blijf ophalen zolang de volgende pagina bestaat.

```typescript
import { generate, of, Observable, concatMap, delay } from 'rxjs';
interface PageState {
  page: number;
  hasNext: boolean;
}

interface PageData {
  page: number;
  items: string[];
  hasNext: boolean;
}

// Functie om het ophalen van paginagegevens te simuleren
function fetchPage(page: number): Observable<PageData> {
  return of({
    page,
    items: [`Item${page}-1`, `Item${page}-2`, `Item${page}-3`],
    hasNext: page < 10 // Tot pagina 10
  }).pipe(
    delay(500) // Simuleer API-aanroep
  );
}

// Haal de pagina op zolang deze bestaat (eigenlijk hasNext uit de API-respons halen)
generate<number, PageState>(
  { page: 1, hasNext: true },        // Initiële status
  state => state.hasNext,            // Doorgaan zolang er een volgende pagina is
  state => ({                        // Statusupdate
    page: state.page + 1,
    hasNext: state.page < 10         // Stel dat er tot 10 pagina's zijn
  }),
  state => state.page                // Paginanummer uitgeven
).pipe(
  concatMap(page => fetchPage(page)) // Elke pagina achtereenvolgens ophalen
).subscribe(
  data => console.log(`Pagina ${data.page} opgehaald:`, data.items),
  err => console.error('Fout:', err),
  () => console.log('Alle pagina\'s opgehaald')
);

// Output:
// Pagina 1 opgehaald: ['Item1-1', 'Item1-2', 'Item1-3']
// Pagina 2 opgehaald: ['Item2-1', 'Item2-2', 'Item2-3']
// ...
// Pagina 10 opgehaald: ['Item10-1', 'Item10-2', 'Item10-3']
// Alle pagina's opgehaald
```

### 4. Aangepaste timer

Geeft gebeurtenissen uit op onregelmatige intervallen.

```typescript
import { generate, of, concatMap, delay } from 'rxjs';
interface TimerState {
  count: number;
  delay: number;
}

// Timer met geleidelijk toenemende vertraging
generate<string, TimerState>(
  { count: 0, delay: 1000 },         // Initiële status: 1 seconde
  state => state.count < 5,          // Tot 5 keer
  state => ({                        // Statusupdate
    count: state.count + 1,
    delay: state.delay + 500         // Verhoog vertraging met 500 ms
  }),
  state => `Gebeurtenis${state.count + 1}`
).pipe(
  concatMap((message, index) => {
    const delayTime = 1000 + index * 500;
    console.log(`${delayTime}ms wachten voor uitgave`);
    return of(message).pipe(delay(delayTime));
  })
).subscribe(console.log);

// Output:
// 1000ms wachten voor uitgave
// Gebeurtenis1 (na 1 seconde)
// 1500ms wachten voor uitgave
// Gebeurtenis2 (na 2,5 seconden)
// 2000ms wachten voor uitgave
// Gebeurtenis3 (na 4,5 seconden)
// ...
```

### 5. Faculteit berekenen

Wiskundige berekeningen als streams weergeven.

```typescript
import { generate } from 'rxjs';

interface FactorialState {
  n: number;
  result: number;
}

// Bereken faculteit van 5 (5! = 5 × 4 × 3 × 2 × 1 = 120)
generate<number, FactorialState>(
  { n: 5, result: 1 },               // Initiële status
  state => state.n > 0,              // Doorgaan voor n > 0
  state => ({                        // Statusupdate
    n: state.n - 1,
    result: state.result * state.n
  }),
  state => state.result              // Tussenresultaat uitgeven
).subscribe(console.log);

// Output: 5, 20, 60, 120, 120
// (1*5, 5*4, 20*3, 60*2, 120*1)
```

## Vergelijking met andere Creation Functions

### generate() vs range()

```typescript
import { generate, range } from 'rxjs';

// range() - eenvoudige opeenvolgende nummering
range(1, 5).subscribe(console.log);
// Output: 1, 2, 3, 4, 5

// generate() - hetzelfde, maar explicieter
generate(
  1,
  x => x <= 5,
  x => x + 1
).subscribe(console.log);
// Output: 1, 2, 3, 4, 5

// Ware waarde van generate(): complexe stappen
generate(
  1,
  x => x <= 100,
  x => x * 2  // Verhogen met factor 2
).subscribe(console.log);
// Output: 1, 2, 4, 8, 16, 32, 64
// (niet mogelijk met range())
```

### generate() vs defer()

```typescript
import { generate, defer, of } from 'rxjs';

// generate() - lusverwerking
generate(1, x => x <= 3, x => x + 1).subscribe(console.log);
// Output: 1, 2, 3

// defer() - genereren bij abonnement (geen lus)
defer(() => of(1, 2, 3)).subscribe(console.log);
// Output: 1, 2, 3

// Verschil: generate() heeft status, defer alleen lazy evaluatie
```

> [!TIP]
> **Selectiecriteria**:
> - **Eenvoudige opeenvolgende nummers** → `range()`
> - **Complexe voorwaarden of stappen** → `generate()`
> - **Dynamisch bepaald bij abonnement** → `defer()`
> - **Fibonacci, faculteit, enz.** → `generate()`

## Asynchronisatie met Scheduler

Bij het verwerken van grote hoeveelheden gegevens is asynchrone uitvoering mogelijk door een scheduler op te geven.

```typescript
import { generate, asyncScheduler, observeOn } from 'rxjs';
console.log('Start');

// Een miljoen lussen asynchroon uitvoeren
generate(
  1,
  x => x <= 1000000,
  x => x + 1
).pipe(
  observeOn(asyncScheduler)
).subscribe({
  next: val => {
    if (val % 100000 === 0) {
      console.log(`Voortgang: ${val}`);
    }
  },
  complete: () => console.log('Voltooid')
});

console.log('Na abonnement (asynchroon, dus wordt onmiddellijk uitgevoerd)');

// Output:
// Start
// Na abonnement (asynchroon, dus wordt onmiddellijk uitgevoerd)
// Voortgang: 100000
// Voortgang: 200000
// ...
// Voltooid
```

## Prestatieoverwegingen

Omdat `generate()` waarden synchroon uitgeeft, moet rekening worden gehouden met prestaties bij het genereren van grote aantallen waarden of het uitvoeren van complexe berekeningen.

> [!WARNING]
> **Prestatie-optimalisatie**:
> ```typescript
> // ❌ Slecht voorbeeld: complexe berekening synchroon uitgevoerd (UI geblokkeerd)
> generate(
>   1,
>   x => x <= 1000000,
>   x => expensiveCalculation(x)
> ).subscribe(console.log);
>
> // ✅ Goed voorbeeld 1: asynchroon met scheduler
> generate(
>   1,
>   x => x <= 1000000,
>   x => expensiveCalculation(x)
> ).pipe(
>   observeOn(asyncScheduler)
> ).subscribe(console.log);
>
> // ✅ Goed voorbeeld 2: Aantal beperken met take()
> generate(
>   1,
>   x => true,  // Oneindige lus
>   x => x + 1
> ).pipe(
>   take(100)   // Alleen de eerste 100
> ).subscribe(console.log);
> ```

## Foutafhandeling

Hoewel `generate()` zelf geen fouten uitgeeft, kunnen fouten optreden in pipelines en statusupdatefuncties.

```typescript
import { generate, of, map, catchError } from 'rxjs';
generate(
  1,
  x => x <= 10,
  x => x + 1
).pipe(
  map(n => {
    if (n === 5) {
      throw new Error('Fout bij 5');
    }
    return n * 2;
  }),
  catchError(error => {
    console.error('Fout opgetreden:', error.message);
    return of(-1); // Standaardwaarde retourneren
  })
).subscribe(console.log);

// Output: 2, 4, 6, 8, -1
```

### Fout in statusupdatefunctie

Een fout binnen een statusupdatefunctie zorgt ervoor dat Observable in een foutstatus komt.

```typescript
import { generate, EMPTY, catchError } from 'rxjs';
generate(
  1,
  x => x <= 10,
  x => {
    if (x === 5) {
      throw new Error('Fout bij statusupdate');
    }
    return x + 1;
  }
).pipe(
  catchError(error => {
    console.error('Fout:', error.message);
    return EMPTY; // Lege Observable retourneren
  })
).subscribe({
  next: console.log,
  complete: () => console.log('Voltooid')
});

// Output: 1, 2, 3, 4, Fout: Fout bij statusupdate, Voltooid
```

## Type-veiligheid in TypeScript

`generate()` kan het type van de status scheiden van het type van de uitgegeven waarde.

```typescript
import { generate } from 'rxjs';

interface State {
  count: number;
  sum: number;
}

interface Result {
  index: number;
  average: number;
}

// Status: State, uitgegeven waarde: Result
const stats$ = generate<Result, State>(
  { count: 1, sum: 0 },
  state => state.count <= 5,
  state => ({
    count: state.count + 1,
    sum: state.sum + state.count
  }),
  state => ({
    index: state.count,
    average: state.sum / state.count
  })
);

stats$.subscribe(result => {
  console.log(`[${result.index}] Gemiddelde: ${result.average}`);
});

// Output:
// [1] Gemiddelde: 0
// [2] Gemiddelde: 0.5
// [3] Gemiddelde: 1
// [4] Gemiddelde: 1.5
// [5] Gemiddelde: 2
```

## Samenvatting

`generate()` is een krachtige Creation Function waarmee complexe lusverwerking declaratief kan worden beschreven.

> [!IMPORTANT]
> **Kenmerken van generate()**:
> - ✅ Flexibele luscontrole zoals while/for-statements
> - ✅ Complex statusbeheer mogelijk
> - ✅ Ideaal voor wiskundige berekeningen zoals Fibonacci, faculteit, enz.
> - ✅ Status en uitgavewaarden kunnen worden gescheiden
> - ⚠️ Pas op voor oneindige lussen (beperkt door `take()`)
> - ⚠️ Overweeg asynchroon voor grote hoeveelheden gegevens
> - ⚠️ Gebruik `range()` voor eenvoudige opeenvolgende nummers

## Gerelateerde onderwerpen

- [range()](/nl/guide/creation-functions/loop/range) - Eenvoudige opeenvolgende nummergeneratie
- [defer()](/nl/guide/creation-functions/conditional/defer) - Dynamische generatie bij abonnement
- [expand()](/nl/guide/operators/transformation/expand) - Recursieve uitbreiding (higher-order operator)
- [scan()](/nl/guide/operators/transformation/scan) - Cumulatieve berekening

## Referenties

- [RxJS Officieel: generate()](https://rxjs.dev/api/index/function/generate)
- [Learn RxJS: generate](https://www.learnrxjs.io/learn-rxjs/operators/creation/generate)
