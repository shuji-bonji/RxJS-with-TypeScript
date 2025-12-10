---
description: partition è una Funzione di Creazione RxJS che divide un Observable in due Observable basandosi su condizioni. È ideale per elaborazione a biforcazione come successo/fallimento, valido/invalido, ecc.
---

# partition - dividi in due stream su condizione

`partition` è una Funzione di Creazione che **divide** un Observable in due Observable basandosi su una condizione.
Puoi specificare la condizione con una funzione predicato (predicate) e ottenere i valori che soddisfano la condizione e i valori che non la soddisfano come stream separati.

## Sintassi e utilizzo base

```ts
import { partition, of } from 'rxjs';

const source$ = of(1, 2, 3, 4, 5, 6);

// Dividi in numeri pari e dispari
const [evens$, odds$] = partition(source$, (value) => value % 2 === 0);

evens$.subscribe((value) => console.log('Pari:', value));
// Output: Pari: 2, Pari: 4, Pari: 6

odds$.subscribe((value) => console.log('Dispari:', value));
// Output: Dispari: 1, Dispari: 3, Dispari: 5
```

- `partition` restituisce un **array contenente due Observable**.
- `[0]`: uno stream di valori che soddisfano la condizione.
- `[1]`: uno stream di valori che non soddisfano la condizione.

[🌐 Documentazione Ufficiale RxJS - `partition`](https://rxjs.dev/api/index/function/partition)

## Pattern di utilizzo tipici

- **Elaborazione separata di successo/fallimento** (ordinamento per codice di stato HTTP)
- **Classificazione eventi** (click sinistro/click destro)
- **Classificazione dati** (valido/invalido, adulto/bambino, ecc.)
- **Divisione stream basata su condizioni**.

## Esempi di codice pratici (con UI)

Quando viene cliccato un pulsante, il processo si ramifica a seconda che le coordinate del click siano per la metà sinistra o destra dello schermo.

```ts
import { partition, fromEvent } from 'rxjs';
import { map } from 'rxjs';

// Crea area output
const leftArea = document.createElement('div');
leftArea.innerHTML = '<h3>Click Sinistro</h3><ul id="left-list"></ul>';
leftArea.style.float = 'left';
leftArea.style.width = '45%';
leftArea.style.padding = '10px';
leftArea.style.background = '#e3f2fd';
document.body.appendChild(leftArea);

const rightArea = document.createElement('div');
rightArea.innerHTML = '<h3>Click Destro</h3><ul id="right-list"></ul>';
rightArea.style.float = 'right';
rightArea.style.width = '45%';
rightArea.style.padding = '10px';
rightArea.style.background = '#fce4ec';
document.body.appendChild(rightArea);

// Eventi click
const clicks$ = fromEvent<MouseEvent>(document, 'click');

// Coordinata X centrale dello schermo
const centerX = window.innerWidth / 2;

// Dividi in metà sinistra e destra
const [leftClicks$, rightClicks$] = partition(
  clicks$,
  (event) => event.clientX < centerX
);

// Elabora click sinistri
leftClicks$.pipe(
  map(event => ({ x: event.clientX, y: event.clientY }))
).subscribe((pos) => {
  const leftList = document.getElementById('left-list')!;
  const li = document.createElement('li');
  li.textContent = `Posizione: (${pos.x}, ${pos.y})`;
  leftList.appendChild(li);
});

// Elabora click destri
rightClicks$.pipe(
  map(event => ({ x: event.clientX, y: event.clientY }))
).subscribe((pos) => {
  const rightList = document.getElementById('right-list')!;
  const li = document.createElement('li');
  li.textContent = `Posizione: (${pos.x}, ${pos.y})`;
  rightList.appendChild(li);
});
```

- Cliccando sullo schermo verrà registrato nelle liste sinistra e destra secondo la posizione del click.
- Due stream indipendenti possono essere creati da un'unica sorgente.

## Esempio pratico: Elaborazione ramificata delle risposte API

Esempio di divisione successo e fallimento per codice di stato HTTP

```ts
import { partition, from, of } from 'rxjs';
import { mergeMap, map, catchError, share } from 'rxjs';

interface ApiResponse {
  status: number;
  data?: any;
  error?: string;
}

// Chiamate API fittizie
const apiCalls$ = from([
  fetch('/api/users/1'),
  fetch('/api/users/999'), // Utente inesistente
  fetch('/api/users/2'),
]);

// Elabora Response e converti in ApiResponse
const responses$ = apiCalls$.pipe(
  mergeMap(fetchPromise => from(fetchPromise)),
  mergeMap(response =>
    from(response.json()).pipe(
      map(data => ({
        status: response.status,
        data: response.ok ? data : undefined,
        error: response.ok ? undefined : (data.message || 'Errore')
      } as ApiResponse)),
      catchError(err => of({
        status: response.status,
        data: undefined,
        error: err.message || 'Impossibile analizzare la risposta'
      } as ApiResponse))
    )
  ),
  share() // Gestisci 2 subscription da partition
);

// Dividi in successo (200s) e fallimento (altri)
const [success$, failure$] = partition(
  responses$,
  (response: ApiResponse) => response.status >= 200 && response.status < 300
);

// Gestisci risposte di successo
success$.subscribe((response) => {
  console.log('✅ Successo:', response.data);
  // Visualizza dati successo nella UI
});

// Gestisci risposte di fallimento
failure$.subscribe((response) => {
  console.error('❌ Fallimento:', response.error);
  // Visualizza messaggio di errore
});
```

## Confronto con filter

### Differenze base

| Metodo | Descrizione | Output | Caso d'Uso |
|--------|-------------|--------|----------|
| `partition` | Dividi una sorgente in due stream | 2 Observable | Quando vuoi usare entrambi gli stream **simultaneamente** |
| `filter` | Passa solo valori che soddisfano la condizione | 1 Observable | Quando serve solo uno stream |

### Esempi di utilizzo

**Usa partition per elaborare entrambi gli stream simultaneamente**

```ts
import { partition, interval } from 'rxjs';
import { map, take } from 'rxjs';

const output = document.createElement('div');
document.body.appendChild(output);

const successArea = document.createElement('div');
successArea.innerHTML = '<h4 style="color: green;">✅ Successo</h4><ul id="success-list"></ul>';
successArea.style.float = 'left';
successArea.style.width = '45%';
output.appendChild(successArea);

const failureArea = document.createElement('div');
failureArea.innerHTML = '<h4 style="color: red;">❌ Fallimento</h4><ul id="failure-list"></ul>';
failureArea.style.float = 'right';
failureArea.style.width = '45%';
output.appendChild(failureArea);

// Stream successo/fallimento casuale
const tasks$ = interval(1000).pipe(
  take(10),
  map(i => ({
    id: i + 1,
    success: Math.random() > 0.5,
    message: `Task ${i + 1}`
  }))
);

// ✅ partition - gestisci successo e fallimento simultaneamente
const [success$, failure$] = partition(tasks$, task => task.success);

success$.subscribe(task => {
  const successList = document.getElementById('success-list')!;
  const li = document.createElement('li');
  li.textContent = task.message;
  successList.appendChild(li);
});

failure$.subscribe(task => {
  const failureList = document.getElementById('failure-list')!;
  const li = document.createElement('li');
  li.textContent = task.message;
  failureList.appendChild(li);
});
```

**Usa filter se serve solo uno stream**

```ts
import { interval } from 'rxjs';
import { map, take, filter } from 'rxjs';

const output = document.createElement('div');
document.body.appendChild(output);

const successArea = document.createElement('div');
successArea.innerHTML = '<h4 style="color: green;">✅ Visualizza solo successo</h4><ul id="success-only"></ul>';
output.appendChild(successArea);

const tasks$ = interval(1000).pipe(
  take(10),
  map(i => ({
    id: i + 1,
    success: Math.random() > 0.5,
    message: `Task ${i + 1}`
  }))
);

// ✅ filter - elabora solo successo (ignora fallimenti)
tasks$
  .pipe(filter(task => task.success))
  .subscribe(task => {
    const successList = document.getElementById('success-only')!;
    const li = document.createElement('li');
    li.textContent = task.message;
    successList.appendChild(li);
  });
```

**Usare filter due volte vs. partition**

```ts
import { of } from 'rxjs';
import { filter } from 'rxjs';
import { partition } from 'rxjs';

const numbers$ = of(1, 2, 3, 4, 5, 6, 7, 8, 9, 10);

// ❌ Usa filter due volte - la sorgente potrebbe essere eseguita due volte
const evens1$ = numbers$.pipe(filter(n => n % 2 === 0));
const odds1$ = numbers$.pipe(filter(n => n % 2 !== 0));

evens1$.subscribe(n => console.log('Pari:', n));
odds1$.subscribe(n => console.log('Dispari:', n));
// Problema: se numbers$ è un cold observable, verrà eseguito due volte

// ✅ Usa partition - crea entrambi gli stream in una singola esecuzione
const [evens2$, odds2$] = partition(numbers$, n => n % 2 === 0);

evens2$.subscribe(n => console.log('Pari:', n));
odds2$.subscribe(n => console.log('Dispari:', n));
// Vantaggio: crea efficientemente due stream da una sorgente
```

**Usa filter se vuoi ramificare nella pipeline**

```ts
import { from } from 'rxjs';
import { filter, map } from 'rxjs';

interface User {
  id: number;
  name: string;
  age: number;
  isActive: boolean;
}

const users$ = from([
  { id: 1, name: 'Alice', age: 25, isActive: true },
  { id: 2, name: 'Bob', age: 30, isActive: false },
  { id: 3, name: 'Carol', age: 35, isActive: true }
]);

// ❌ partition è una Funzione di Creazione, quindi non può essere usato in una pipeline
// users$.pipe(
//   map(user => user.name),
//   partition(name => name.startsWith('A')) // Errore
// );

// ✅ Usa filter - disponibile nella pipeline
users$
  .pipe(
    filter(user => user.isActive),  // Solo utenti attivi
    map(user => user.name)           // Estrai nome
  )
  .subscribe(console.log);
// Output: Alice, Carol
```

### Riepilogo

| Situazione | Metodo Raccomandato | Motivo |
|-----------|-------------------|--------|
| Vuoi elaborare **entrambi** successo e fallimento | `partition` | Può creare due stream in una singola esecuzione della sorgente |
| Vuoi elaborare **solo** successo | `filter` | Semplice e chiaro |
| Vuoi ramificare condizioni nella pipeline | `filter` | `partition` non può essere usato perché è una Funzione di Creazione |
| Vuoi ramificare in 3 o più con condizioni complesse | `groupBy` | Può dividere in gruppi multipli |

## Note

### 1. Sottoscrivi entrambi gli stream

I due Observable creati in una `partition` **condividono** la sorgente originale.
Se non ti sottoscrivi a entrambi, lo stream originale potrebbe non essere elaborato completamente.

```ts
const [success$, failure$] = partition(source$, predicate);

// Sottoscrivi entrambi
success$.subscribe(handleSuccess);
failure$.subscribe(handleFailure);
```

### 2. La sorgente viene eseguita due volte

`partition` internamente si sottoscrive alla sorgente originale due volte.
Fai attenzione agli effetti collaterali.

```ts
let callCount = 0;
const source$ = new Observable(observer => {
  callCount++;
  console.log(`Conteggio subscription: ${callCount}`);
  observer.next(1);
  observer.complete();
});

const [a$, b$] = partition(source$, n => n > 0);
a$.subscribe(); // Conteggio subscription: 1
b$.subscribe(); // Conteggio subscription: 2
```

Per evitare effetti collaterali, usa `share()`.

```ts
import { share } from 'rxjs';

const shared$ = source$.pipe(share());
const [a$, b$] = partition(shared$, n => n > 0);
```

### 3. Non fornito come Pipeable Operator

Da RxJS 7, `partition` è fornito **solo come Funzione di Creazione**.
Non può essere usato all'interno di una pipeline.

```ts
// ❌ Non possibile
source$.pipe(
  partition(n => n % 2 === 0) // Errore
);

// ✅ Utilizzo corretto
const [evens$, odds$] = partition(source$, n => n % 2 === 0);
```

## Pattern Alternativi

Se vuoi ramificare all'interno di una pipeline, usa `filter`.

```ts
const source$ = of(1, 2, 3, 4, 5, 6);

const evens$ = source$.pipe(filter(n => n % 2 === 0));
const odds$ = source$.pipe(filter(n => n % 2 !== 0));

// Oppure condividi la sorgente con share
const shared$ = source$.pipe(share());
const evens$ = shared$.pipe(filter(n => n % 2 === 0));
const odds$ = shared$.pipe(filter(n => n % 2 !== 0));
```

## Operatori correlati

- [`filter`](../../operators/filtering/filter.md) - passa solo valori che soddisfano una condizione
- [`groupBy`](../../operators/transformation/groupBy.md) - Dividi in gruppi multipli
- [`share`](../../operators/multicasting/share.md) - Condividi una sorgente

## Riepilogo

`partition` è uno strumento potente per dividere un Observable in due basandosi su una condizione.

- ✅ Ideale per elaborazione separata successo/fallimento
- ✅ Crea due stream indipendenti
- ⚠️ Le sorgenti vengono sottoscritte due volte (nota gli effetti collaterali)
- ⚠️ Non offerto come Pipeable Operator
