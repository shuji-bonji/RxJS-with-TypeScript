---
description: raceWith è un operatore di combinazione RxJS che adotta solo il primo stream che emette un valore tra l'Observable originale e altri Observable. È la versione Pipeable Operator della Funzione di Creazione race, ed è ideale per situazioni in cui la risposta più veloce ha priorità, come implementazioni di timeout, acquisizione parallela da più CDN (fallback), e acquisizione competitiva da più sorgenti dati. È efficace quando vuoi competere con altri stream mentre converti ed elabori lo stream principale.
titleTemplate: ':title | RxJS'
---

# raceWith - Più Veloce Vince

L'operatore `raceWith` **adotta solo il primo stream che emette un valore** tra l'Observable originale e gli altri Observable specificati, e ignora tutti gli altri.
Questa è la versione Pipeable Operator della Funzione di Creazione `race`.

## 🔰 Sintassi e Utilizzo Base

```ts
import { interval, timer } from 'rxjs';
import { raceWith, map, take } from 'rxjs';

const source1$ = interval(1000).pipe(
  take(3),
  map(val => `Sorgente 1: ${val}`)
);

const source2$ = timer(500).pipe(
  take(3),
  map(val => `Sorgente 2: ${val}`)
);

source1$
  .pipe(raceWith(source2$))
  .subscribe(console.log);

// Output:
// Sorgente 2: 0 (dopo 500ms)
// * source1$ viene ignorato perché source2$ ha emesso per primo
```

- **Il primo Observable a emettere un valore** vince la gara, e solo quello stream viene adottato.
- Gli altri Observable vengono automaticamente annullati e ignorati.

[🌐 Documentazione Ufficiale RxJS - `raceWith`](https://rxjs.dev/api/operators/raceWith)


## 💡 Pattern di Utilizzo Tipici

- **Implementazione timeout**: Compete l'elaborazione principale con errore di timeout dopo un certo tempo
- **Acquisizione parallela da più CDN**: Richiedi più CDN simultaneamente e adotta la risposta più veloce (strategia fallback)
- **Acquisizione competitiva da più sorgenti dati**: Esegui cache locale e chiamata API concorrentemente, e usa qualunque restituisca per prima
- **Azione utente vs competizione timer**: Compete azione click con avanzamento automatico, e adotta qualunque si verifichi per primo


## 🧠 Esempio di Codice Pratico (con UI)

Esempio di recupero dati da più CDN in parallelo e adozione della risposta più veloce.

```ts
import { fromFetch } from 'rxjs/fetch';
import { raceWith, map, catchError, timeout } from 'rxjs';
import { of } from 'rxjs';

// Costruisci la UI
const container = document.createElement('div');
container.innerHTML = `
  <h3>Esempio Pratico raceWith: Fetch Parallelo da Più CDN</h3>
  <button id="fetch-button">Recupera Dati</button>
  <div id="status" style="margin-top: 10px; padding: 10px; border: 1px solid #ccc;">
    In attesa...
  </div>
  <div id="result" style="margin-top: 10px;"></div>
`;
document.body.appendChild(container);

const fetchButton = document.getElementById('fetch-button') as HTMLButtonElement;
const statusDiv = document.getElementById('status')!;
const resultDiv = document.getElementById('result')!;

// Inizia recupero dati al click del pulsante
fetchButton.addEventListener('click', () => {
  statusDiv.textContent = 'Recupero da più CDN in parallelo...';
  statusDiv.style.backgroundColor = '#fff3e0';
  resultDiv.innerHTML = '';

  // Più CDN (in realtà endpoint fittizi)
  const cdn1$ = fromFetch('https://jsonplaceholder.typicode.com/posts/1').pipe(
    map(response => response.json()),
    map(() => ({ source: 'CDN 1', data: 'Dati recuperati con successo' })),
    timeout(3000),
    catchError(() => of({ source: 'CDN 1', data: 'Errore' }))
  );

  const cdn2$ = fromFetch('https://jsonplaceholder.typicode.com/posts/2').pipe(
    map(response => response.json()),
    map(() => ({ source: 'CDN 2', data: 'Dati recuperati con successo' })),
    timeout(3000),
    catchError(() => of({ source: 'CDN 2', data: 'Errore' }))
  );

  const cdn3$ = fromFetch('https://jsonplaceholder.typicode.com/posts/3').pipe(
    map(response => response.json()),
    map(() => ({ source: 'CDN 3', data: 'Dati recuperati con successo' })),
    timeout(3000),
    catchError(() => of({ source: 'CDN 3', data: 'Errore' }))
  );

  // ✅ Adotta la risposta più veloce con raceWith
  cdn1$
    .pipe(raceWith(cdn2$, cdn3$))
    .subscribe({
      next: (result) => {
        statusDiv.textContent = `✅ Recuperato con successo da ${result.source}`;
        statusDiv.style.backgroundColor = '#e8f5e9';
        resultDiv.innerHTML = `<strong>${result.source}</strong>: ${result.data}`;
      },
      error: (err) => {
        statusDiv.textContent = '❌ Impossibile recuperare da tutti i CDN';
        statusDiv.style.backgroundColor = '#ffebee';
        resultDiv.textContent = `Errore: ${err.message}`;
      }
    });
});
```

- Richiede più CDN simultaneamente, e **adotta il primo CDN** che restituisce una risposta.
- Le risposte dagli altri CDN vengono automaticamente ignorate.


## 🔄 Differenza dalla Funzione di Creazione `race`

### Differenze Base

| | `race` (Funzione di Creazione) | `raceWith` (Pipeable Operator) |
|:---|:---|:---|
| **Posizione di Utilizzo** | Usato come funzione indipendente | Usato all'interno della catena `.pipe()` |
| **Sintassi** | `race(obs1$, obs2$, obs3$)` | `obs1$.pipe(raceWith(obs2$, obs3$))` |
| **Primo Stream** | Tratta tutti ugualmente | Tratta come stream principale |
| **Vantaggio** | Semplice e leggibile | Facile da combinare con altri operatori |

### Esempi di Utilizzo Specifici

**La Funzione di Creazione è Raccomandata per Sola Competizione Semplice**

```ts
import { race, timer } from 'rxjs';
import { map } from 'rxjs';

const fast$ = timer(100).pipe(map(() => 'Veloce vince!'));
const slow$ = timer(500).pipe(map(() => 'Lento vince!'));

// Semplice e leggibile
race(fast$, slow$).subscribe(console.log);
// Output: Veloce vince!
```

**Il Pipeable Operator è Raccomandato Quando si Aggiunge Elaborazione di Trasformazione allo Stream Principale**

```ts
import { fromEvent, timer } from 'rxjs';
import { raceWith, map, mapTo, take } from 'rxjs';

// Click utente vs competizione avanzamento automatico
const userClick$ = fromEvent(document, 'click').pipe(
  take(1),
  mapTo('L\'utente ha cliccato')
);

const autoAdvance$ = timer(5000).pipe(
  mapTo('Avanzamento automatico')
);

// ✅ Versione Pipeable Operator - aggiungi elaborazione allo stream principale
userClick$
  .pipe(
    map(message => `[${new Date().toLocaleTimeString()}] ${message}`),
    raceWith(autoAdvance$.pipe(
      map(message => `[${new Date().toLocaleTimeString()}] ${message}`)
    ))
  )
  .subscribe(console.log);

// ❌ Versione Funzione di Creazione - diventa prolissa
import { race } from 'rxjs';
race(
  userClick$.pipe(
    map(message => `[${new Date().toLocaleTimeString()}] ${message}`)
  ),
  autoAdvance$.pipe(
    map(message => `[${new Date().toLocaleTimeString()}] ${message}`)
  )
).subscribe(console.log);
```

### Riepilogo

- **`race`**: Ottimale per competere semplicemente più stream
- **`raceWith`**: Ottimale quando vuoi competere con altri stream mentre trasformi o elabori lo stream principale


## ⚠️ Note Importanti

### La Prima Emissione Vince

Lo stream con il **timing di emissione più precoce** viene adottato. Non il timing di inizio subscription.

```ts
import { timer, of } from 'rxjs';
import { raceWith, map } from 'rxjs';

const immediate$ = of('Emetti immediatamente');
const delayed$ = timer(1000).pipe(map(() => 'Emetti dopo 1 secondo'));

immediate$
  .pipe(raceWith(delayed$))
  .subscribe(console.log);
// Output: Emetti immediatamente
```

### Tutti gli Observable Vengono Sottoscritti

`raceWith` **sottoscrive tutti gli Observable simultaneamente**, ma ignora tutti tranne il primo che emette.

```ts
import { timer } from 'rxjs';
import { raceWith, tap } from 'rxjs';

const source1$ = timer(100).pipe(
  tap(() => console.log('Sorgente 1 emette'))
);

const source2$ = timer(200).pipe(
  tap(() => console.log('Sorgente 2 emette'))
);

source1$
  .pipe(raceWith(source2$))
  .subscribe(console.log);
// Output:
// Sorgente 1 emette
// 0
// Sorgente 2 emette ← Sottoscritto, ma il valore viene ignorato
```

### Gestione Errori

Se c'è un Observable che emette un errore per primo, l'intero stream termina con un errore.

```ts
import { throwError, timer } from 'rxjs';
import { raceWith, catchError } from 'rxjs';
import { of } from 'rxjs';

timer(1000).pipe(
  raceWith(
    throwError(() => new Error('Si è verificato un errore')).pipe(
      catchError(err => of('Errore recuperato'))
    )
  )
).subscribe({
  next: console.log,
  error: err => console.error('Errore:', err.message)
});
// Output: Errore recuperato
```


## 📚 Operatori Correlati

- **[race](/it/guide/creation-functions/selection/race)** - Versione Funzione di Creazione
- **[mergeWith](/it/guide/operators/combination/mergeWith)** - Esegui tutti gli stream in parallelo
- **[concatWith](/it/guide/operators/combination/concatWith)** - Esegui stream sequenzialmente
