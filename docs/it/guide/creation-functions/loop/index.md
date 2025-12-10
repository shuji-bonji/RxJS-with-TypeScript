---
description: Questa sezione descrive le Funzioni di Creazione che generano valori in modo simile a un loop, usando range e generate per imparare come implementare elaborazione iterativa come istruzioni for e while come stream Observable. Dalla generazione di numeri sequenziali a transizioni di stato complesse basate su condizioni personalizzate, puoi realizzare elaborazione di loop dichiarativa sfruttando l'inferenza di tipo di TypeScript.
---

# Funzioni di Creazione Generazione Loop

Funzioni di Creazione per esprimere elaborazione di loop come istruzioni for e while come Observable.

## Cosa Sono le Funzioni di Creazione Generazione Loop?

Le Funzioni di Creazione Generazione Loop realizzano reattivamente elaborazione ripetitiva. Sostituendo i loop imperativi convenzionali (istruzioni `for` e `while`) con stream Observable dichiarativi, è possibile un'elaborazione flessibile in combinazione con la catena di operatori RxJS.

Controlla la tabella sotto per vedere le caratteristiche e l'utilizzo di ogni Funzione di Creazione.

## Principali Funzioni di Creazione Generazione Loop

| Funzione | Descrizione | Casi d'Uso |
|----------|------|-------------|
| **[range](/it/guide/creation-functions/loop/range)** | Genera un range di numeri (come istruzione for) | Generazione numeri sequenziali, elaborazione batch |
| **[generate](/it/guide/creation-functions/loop/generate)** | Generazione loop generica (come istruzione while) | Ripetizione condizionale, transizioni stato complesse |

## Criteri di Utilizzo

La selezione delle Funzioni di Creazione Generazione Loop è determinata dalle seguenti prospettive.

### 1. Pattern di Generazione

- **Sequenza numerica**: `range()` - Semplice generazione numeri sequenziali con valori inizio e fine
- **Condizioni complesse**: `generate()` - Controllo libero su valori iniziali, condizioni, iterazione e selezione risultato

### 2. Tipi di Loop

- **Loop simile a istruzione for**: `range()` - `for (let i = start; i <= end; i++)`
- **Loop simile a istruzione while**: `generate()` - `while (condition) { ... }`

### 3. Flessibilità

- **Semplice è sufficiente**: `range()` - Quando serve una sequenza di numeri
- **Serve controllo avanzato**: `generate()` - Gestione stato personalizzata, ramificazione condizionale, controllo step

## Esempi di Utilizzo Pratico

### range() - Generazione Numeri Sequenziali

Per semplice generazione numeri sequenziali, `range()` è la scelta migliore.

```typescript
import { range, map } from 'rxjs';
// Genera numeri sequenziali da 1 a 5
range(1, 5).subscribe(console.log);
// Output: 1, 2, 3, 4, 5

// Usa in elaborazione batch
range(0, 10).pipe(
  map(i => `Processo ${i + 1}`)
).subscribe(console.log);
// Output: Processo 1, Processo 2, ..., Processo 10
```

### generate() - Loop Condizionale

Usa `generate()` per condizioni complesse o gestione stato personalizzata.

```typescript
import { generate } from 'rxjs';

// Genera sequenza Fibonacci (primi 10 termini)
generate(
  { current: 0, next: 1, count: 0 },  // Stato iniziale
  state => state.count < 10,           // Condizione continuazione
  state => ({                          // Aggiornamento stato
    current: state.next,
    next: state.current + state.next,
    count: state.count + 1
  }),
  state => state.current               // Selettore risultato
).subscribe(console.log);
// Output: 0, 1, 1, 2, 3, 5, 8, 13, 21, 34
```

## Confronto con Loop Imperativo

Questo è un confronto tra il loop imperativo convenzionale e le Funzioni di Creazione Generazione Loop di RxJS.

### Istruzione for Imperativa

```typescript
// Istruzione for convenzionale
const results: number[] = [];
for (let i = 1; i <= 5; i++) {
  results.push(i * 2);
}
console.log(results); // [2, 4, 6, 8, 10]
```

### range() Dichiarativo

```typescript
import { range, map, toArray } from 'rxjs';
// RxJS range()
range(1, 5).pipe(
  map(i => i * 2),
  toArray()
).subscribe(console.log); // [2, 4, 6, 8, 10]
```

> [!TIP]
> **Vantaggi dell'approccio dichiarativo**:
> - Leggibilità migliorata con elaborazione pipeline
> - Gestione errori uniforme
> - Facile da combinare con elaborazione asincrona
> - Facile da annullare e interrompere (es., `takeUntil()`)

## Conversione da Cold a Hot

Come mostrato nella tabella sopra, **tutte le Funzioni di Creazione Generazione Loop generano Cold Observable**. Ogni subscription avvia un'esecuzione indipendente.

Tuttavia, usando operatori multicast (`share()`, `shareReplay()`, ecc.), puoi **convertire un Cold Observable in Hot Observable**.

### Esempio Pratico: Condivisione Risultati Calcolo

```typescript
import { range, map, share } from 'rxjs';
// ❄️ Cold - Calcolo indipendente per ogni subscription
const cold$ = range(1, 1000).pipe(
  map(n => {
    console.log('Calcolando:', n);
    return n * n;
  })
);

cold$.subscribe(val => console.log('Subscriber 1:', val));
cold$.subscribe(val => console.log('Subscriber 2:', val));
// → Calcolo eseguito due volte (2000 calcoli)

// 🔥 Hot - Condividi risultati calcolo tra subscriber
const hot$ = range(1, 1000).pipe(
  map(n => {
    console.log('Calcolando:', n);
    return n * n;
  }),
  share()
);

hot$.subscribe(val => console.log('Subscriber 1:', val));
hot$.subscribe(val => console.log('Subscriber 2:', val));
// → Calcolo eseguito solo una volta (1000 calcoli)
```

> [!TIP]
> **Casi in cui serve conversione Hot**:
> - Usare calcoli costosi in più posizioni
> - Condividere risultati elaborazione batch con più componenti
> - Visualizzare risultati paginazione in più componenti UI
>
> Per maggiori informazioni, vedi [Creazione Base - Conversione da Cold a Hot](/it/guide/creation-functions/basic/#conversione-da-cold-a-hot).

## Combinato con Elaborazione Asincrona

Le Funzioni di Creazione Generazione Loop dimostrano funzionalità potenti quando combinate con elaborazione asincrona.

### Esecuzione Sequenziale di Chiamate API

```typescript
import { range, of, Observable, concatMap, delay } from 'rxjs';
interface PageData {
  page: number;
  items: string[];
}

// Funzione per simulare recupero dati pagina
function fetchPage(page: number): Observable<PageData> {
  return of({
    page,
    items: [`Data${page}-1`, `Data${page}-2`, `Data${page}-3`]
  }).pipe(
    delay(300) // Simula chiamata API
  );
}

// Recupera sequenzialmente pagine da 1 a 10 (con delay 1 secondo tra ogni richiesta)
range(1, 10).pipe(
  concatMap(page =>
    fetchPage(page).pipe(delay(1000))
  )
).subscribe(
  data => console.log(`Pagina ${data.page} recuperata:`, data.items),
  err => console.error('Errore:', err)
);
```

### Uso in Elaborazione Retry

```typescript
import { range, throwError, of, Observable, mergeMap, retryWhen, delay } from 'rxjs';
// Funzione per simulare recupero dati (fallisce casualmente)
function fetchData(): Observable<string> {
  const shouldFail = Math.random() > 0.6; // 40% tasso successo

  return of(shouldFail).pipe(
    delay(200),
    mergeMap(fail =>
      fail
        ? throwError(() => new Error('Recupero dati fallito'))
        : of('Recupero dati riuscito')
    )
  );
}

function fetchWithRetry() {
  return fetchData().pipe(
    retryWhen(errors =>
      errors.pipe(
        mergeMap((error, index) => {
          // Riprova fino a 3 volte
          if (index >= 3) {
            return throwError(() => error);
          }
          console.log(`Retry ${index + 1}/3`);
          // Backoff esponenziale: 1s, 2s, 4s
          return range(0, 1).pipe(delay(Math.pow(2, index) * 1000));
        })
      )
    )
  );
}

fetchWithRetry().subscribe({
  next: result => console.log('Risultato:', result),
  error: err => console.error('Errore:', err.message)
});

// Esempio output:
// Retry 1/3
// Retry 2/3
// Risultato: Recupero dati riuscito
```

## Relazione con Pipeable Operator

Le Funzioni di Creazione Generazione Loop non hanno un corrispettivo Pipeable Operator diretto. Vengono sempre usate come Funzioni di Creazione.

Tuttavia, è possibile un'elaborazione più avanzata combinandole con i seguenti operatori:

| Operatori da Combinare | Scopo |
|-------------------|------|
| `map()` | Trasforma ogni valore |
| `filter()` | Passa solo valori che corrispondono alla condizione |
| `take()`, `skip()` | Controlla il numero di valori |
| `concatMap()`, `mergeMap()` | Esegui elaborazione asincrona per ogni valore |
| `toArray()` | Raccogli tutti i valori in un array |

## Note sulle Performance

Le Funzioni di Creazione Generazione Loop emettono valori sincronamente, quindi fai attenzione alle performance quando generi un gran numero di valori.

> [!WARNING]
> **Gestione grandi quantità di dati**:
> - Grandi quantità di dati, come `range(1, 1000000)`, vengono tutte emesse sincronamente e consumano memoria
> - Usa buffer con `bufferCount()` o `windowCount()` se necessario
> - Oppure cambia a esecuzione asincrona specificando uno scheduler con `scheduled()`

```typescript
import { range, asyncScheduler, observeOn } from 'rxjs';
// Esegui con scheduler asincrono
range(1, 1000000).pipe(
  observeOn(asyncScheduler)
).subscribe(console.log);
```

## Prossimi Passi

Per apprendere il comportamento dettagliato e gli esempi pratici di ogni Funzione di Creazione, clicca sui link dalla tabella sopra.

Puoi anche comprendere il quadro completo delle Funzioni di Creazione imparando le [Funzioni di Creazione Base](/it/guide/creation-functions/basic/), le [Funzioni di Creazione Combinazione](/it/guide/creation-functions/combination/), le [Funzioni di Creazione Selezione/Partizione](/it/guide/creation-functions/selection/) e le [Funzioni di Creazione Condizionali](/it/guide/creation-functions/conditional/).
