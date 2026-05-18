---
description: "Verranno spiegate le Creation Function che generano valori in modo simile a un ciclo: imparate a usare range e generate per implementare processi iterativi come le istruzioni for e while come flussi Observable. Dalla generazione di numeri sequenziali a complesse transizioni di stato basate su condizioni personalizzate, è possibile realizzare un'elaborazione dichiarativa dei cicli che fa uso dell'inferenza di tipo di TypeScript."
---

# Sistema di generazione di loop Creation Function

Creation Function per rappresentare l'elaborazione di loop, come le istruzioni for e while, come Observable.

## Sistema di generazione di loop Cos'è la Creation Function?

Le Creation Function per i sistemi di generazione di loop realizzano in modo reattivo i processi ripetitivi. Sostituendo i tradizionali loop imperativi (istruzioni `for` e `while`) con flussi dichiarativi Observable, consentono un'elaborazione flessibile in combinazione con la catena di operatori di RxJS.

La tabella seguente illustra le caratteristiche e l'uso di ogni Creation Function.

## Sistemi di generazione di loop principali Creation Function

| Funzione | Descrizione | Caso d'uso. |
|---|---|---|
| **[range](/it/guide/creation-functions/loop/range)** | Genera un intervallo di numeri (per le dichiarazioni). | Generazione di numeri sequenziali, elaborazione batch. |
| **[generate](/it/guide/creation-functions/loop/generate)** | Genera loop generici (tipo while statement) | Ripetizione condizionale, transizioni di stato complesse |

## Criteri di utilizzo

La scelta delle Creation Function per la generazione di loop è determinata dai seguenti aspetti.

### 1. modello di generazione

- **Sequenza di numeri**: `range()` - semplice generazione di numeri sequenziali con valori iniziali e finali.
- **Condizioni complesse**: `generate()` - controllo libero su valori iniziali, condizioni, iterazioni e selezione dei risultati.

### 2. Tipi di loop

- **ciclo simile a un'istruzione for**: `range()` - `for (let i = start; i <= end; i++)`
- **ciclo di tipowhile**: `generate()` - `while (condizione) { ... }`

### 3. flessibilità

- **Bastanza semplice**: `range()` - se è richiesta una sequenza di numeri
- **Necessità di un controllo avanzato**: `generate()` - gestione personalizzata degli stati, ramificazione condizionale, controllo dei passi

## Casi d'uso pratici

### range() - generazione di numeri sequenziali

Per la semplice generazione di numeri sequenziali, range()` è la scelta migliore.

```typescript
import { range, map } from 'rxjs';
// 1a5Generare numeri sequenziali da
range(1, 5).subscribe(console.log);
// Uscita: 1, 2, 3, 4, 5

// Utilizzo nell'elaborazione batch
range(0, 10).pipe(
  map(i => `Elaborazione${i + 1}`)
).subscribe(console.log);
// Uscita: Elaborazione1, Elaborazione2, ..., Elaborazione10
```

### generate() - ciclo condizionale

Usare generate()` quando sono necessarie condizioni complesse o una gestione personalizzata dello stato.

```typescript
import { generate } from 'rxjs';

// Generare la sequenza di Fibonacci (primo10termine)
generate(
  { current: 0, next: 1, count: 0 },  // Condizione iniziale
  state => state.count < 10,           // Condizione di continuazione
  state => ({                          // Aggiornamento dello stato
    current: state.next,
    next: state.current + state.next,
    count: state.count + 1
  }),
  state => state.current               // Selezione del risultato
).subscribe(console.log);
// Uscita: 0, 1, 1, 2, 3, 5, 8, 13, 21, 34
```

## Rispetto al ciclo imperativo

Questo è un confronto tra i loop imperativi convenzionali e il sistema di generazione di loop di RxJS Creation Function.

### Dichiarazione imperativa for

```typescript
// ConvenzionaleforFrase
const results: number[] = [];
for (let i = 1; i <= 5; i++) {
  results.push(i * 2);
}
console.log(results); // [2, 4, 6, 8, 10]
```

### Dichiarativo range()

```typescript
import { range, map, toArray } from 'rxjs';
// RxJSdirange()
range(1, 5).pipe(
  map(i => i * 2),
  toArray()
).subscribe(console.log); // [2, 4, 6, 8, 10]
```

```typescript
import { range, map } from 'rxjs';
// 1a5Generare numeri sequenziali da
range(1, 5).subscribe(console.log);
// Uscita: 1, 2, 3, 4, 5

// Utilizzo nell'elaborazione batch
range(0, 10).pipe(
  map(i => `Elaborazione${i + 1}`)
).subscribe(console.log);
// Uscita: Elaborazione1, Elaborazione2, ..., Elaborazione10
```

> **Benefici dell'approccio dichiarativo**:.
> - Migliore leggibilità grazie all'elaborazione in pipeline
> Gestione uniforme degli errori
> - Facilità di combinazione con l'elaborazione asincrona
> Facilità di annullamento e interruzione (es. takeUntil()`)

## Conversione da freddo a caldo

Come mostrato nella tabella precedente, **tutte le Creation Function che generano loop generano un Observable freddo**. Ogni sottoscrizione avvia un'esecuzione indipendente.

Tuttavia, gli **Osservable freddi possono essere convertiti** in Observable caldi utilizzando gli operatori multicast (`share()`, `shareReplay()`, ecc.).

### Esempio pratico: condivisione dei risultati di un calcolo.


```typescript
import { range, map, share } from 'rxjs';
// ❄️ Cold - Calcolo indipendente per abbonamento
const cold$ = range(1, 1000).pipe(
  map(n => {
    console.log('Calcolo in corso:', n);
    return n * n;
  })
);

cold$.subscribe(val => console.log('Abbonato1:', val));
cold$.subscribe(val => console.log('Abbonato2:', val));
// → Un calcolo viene2eseguito una volta (in2000Calcolo eseguito una volta)

// 🔥 Hot - I risultati del calcolo sono condivisi tra gli abbonati
const hot$ = range(1, 1000).pipe(
  map(n => {
    console.log('Calcolo in corso:', n);
    return n * n;
  }),
  share()
);

hot$.subscribe(val => console.log('Abbonato1:', val));
hot$.subscribe(val => console.log('Abbonato2:', val));
// → Il calcolo viene1eseguito una sola volta (1000Calcolo eseguito una volta)
```

```typescript
import { range, map } from 'rxjs';
// 1a5Generare numeri sequenziali da
range(1, 5).subscribe(console.log);
// Uscita: 1, 2, 3, 4, 5

// Utilizzo nell'elaborazione batch
range(0, 10).pipe(
  map(i => `Elaborazione${i + 1}`)
).subscribe(console.log);
// Uscita: Elaborazione1, Elaborazione2, ..., Elaborazione10
```

> **Casi in cui è richiesto il riscaldamento**:.
> - I calcoli ad alto costo sono utilizzati in più postazioni
> I risultati dell'elaborazione batch sono condivisi da più componenti.
> - I risultati del processo di paginazione vengono visualizzati in più componenti dell'interfaccia utente.
>
> Per ulteriori informazioni, vedere [Sistemi di creazione di base - Conversione da freddo a caldo] (/it/guide/creation-functions/basic/#cold- to -hot-).

## Combinato con l'elaborazione asincrona

Il sistema di generazione di cicli Creation Function può essere combinato con l'elaborazione asincrona per fornire potenti funzionalità.

### Esecuzione sequenziale di chiamate API


```typescript
import { range, of, Observable, concatMap, delay } from 'rxjs';
interface PageData {
  page: number;
  items: string[];
}

// Funzione per simulare l'acquisizione dei dati della pagina
function fetchPage(page: number): Observable<PageData> {
  return of({
    page,
    items: [`Dati${page}-1`, `Dati${page}-2`, `Dati${page}-3`]
  }).pipe(
    delay(300) // APISimula una chiamata a
  );
}

// Pagina1a10Recupera in sequenza fino a (con un ritardo di1secondi di ritardo tra ogni richiesta)
range(1, 10).pipe(
  concatMap(page =>
    fetchPage(page).pipe(delay(1000))
  )
).subscribe({
  next: data => console.log(`Pagina ${data.page} Recupero:`, data.items),
  error: err => console.error('Errore:', err)
});
```

### Utilizzo nell'elaborazione dei tentativi

```typescript
import { range, throwError, of, Observable, mergeMap, retry, delay } from 'rxjs';
// Funzione per simulare l'acquisizione dei dati (fallisce in modo casuale)
function fetchData(): Observable<string> {
  const shouldFail = Math.random() > 0.6; // 40%Successo con una probabilità di

  return of(shouldFail).pipe(
    delay(200),
    mergeMap(fail =>
      fail
        ? throwError(() => new Error('Fallimento dell'acquisizione dei dati'))
        : of('Acquisizione dei dati riuscita')
    )
  );
}

function fetchWithRetry() {
  return fetchData().pipe(
    // RxJS 7.3+ Raccomandazione: retry({ count, delay }) Formato
    retry({
      count: 3, // Max.3Ripetizioni
      delay: (error, retryCount) => {
        console.log(`Ripetizioni ${retryCount}/3`);
        // Arretramento esponenziale: 1Secondi,2Secondi,4sec.
        return range(0, 1).pipe(delay(Math.pow(2, retryCount - 1) * 1000));
      }
    })
  );
}

fetchWithRetry().subscribe({
  next: result => console.log('Risultato:', result),
  error: err => console.error('Errore:', err.message)
});

// Esempio di uscita:
// Ripetizioni 1/3
// Ripetizioni 2/3
// Risultato: Acquisizione dei dati riuscita
```

## Relazione con Pipeable Operator

Le Creation Function che generano loop non hanno una controparte diretta Pipeable Operator. Vengono sempre utilizzate come Creation Function.

Tuttavia, possono essere combinate con i seguenti operatori per un'elaborazione più avanzata.

```typescript
import { range, map } from 'rxjs';
// 1a5Generare numeri sequenziali da
range(1, 5).subscribe(console.log);
// Uscita: 1, 2, 3, 4, 5

// Utilizzo nell'elaborazione batch
range(0, 10).pipe(
  map(i => `Elaborazione${i + 1}`)
).subscribe(console.log);
// Uscita: Elaborazione1, Elaborazione2, ..., Elaborazione10
```

## Note sulle prestazioni.

Le Creation Function che generano loop emettono valori in modo sincrono, quindi è necessario prendere nota delle prestazioni quando si generano un gran numero di valori.

> [!WARNING]

> **Gestione di grandi quantità di dati**:.
> - Grandi quantità di dati, come `range(1, 1000000)`, vengono emesse in modo sincrono e consumano memoria.
> - bufferizzare con `bufferCount()` o `windowCount()` come richiesto
> oppure passare all'esecuzione asincrona specificando uno scheduler con `scheduled()`.


```typescript
import { range, asyncScheduler, observeOn } from 'rxjs';
// Eseguito da uno scheduler asincrono
range(1, 1000000).pipe(
  observeOn(asyncScheduler)
).subscribe(console.log);
```

## Prossimi passi.

Per saperne di più sul funzionamento di Creation Function e sugli esempi pratici, fate clic sui link della tabella precedente.

È inoltre possibile conoscere [Funzioni di creazione di base](/it/guide/creation-functions/basic/), [Funzioni di creazione combinate](/it/guide/creation-functions/combination/), [selezione e Creation Function](/it/guide/creation-functions/selezione/) e [Creation Function](/it/guide/creation-functions/conditional/). Questo vi aiuterà a capire l'intero quadro delle Creation Function.
