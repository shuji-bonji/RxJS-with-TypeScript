---
description: "Descrive una strategia completa di gestione degli errori per RxJS, includendo come combinare gli operatori catchError, retry, retryWhen e finalize, retry con backoff esponenziale, classificazione degli errori e gestione appropriata, gestori di errori globali e come implementare una solida gestione degli errori in TypeScript."
---

# Strategie di gestione degli errori in RxJS

La gestione degli errori in RxJS è un aspetto importante della programmazione reattiva. L'implementazione di una corretta gestione degli errori migliora la robustezza e l'affidabilità dell'applicazione. Questo documento descrive le diverse strategie di gestione degli errori disponibili in RxJS.

## Pattern di base

RxJS gestisce gli errori come parte del ciclo di vita degli Observable. La gestione degli errori di base comprende i seguenti metodi.

```ts
import { of, throwError } from 'rxjs';
import { catchError } from 'rxjs';

// Observable che lancia un errore
const error$ = throwError(() => new Error('Si è verificato un errore')); // RxJS 7 o successivo, formato funzione consigliato

// Gestione degli errori di base
error$
  .pipe(
    catchError((error) => {
      console.error('Errore rilevato:', error.message);
      return of('Valore di fallback dopo l\'errore');
    })
  )
  .subscribe({
    next: (value) => console.log('Valore:', value),
    error: (err) => console.error('Errore non gestito:', err),
    complete: () => console.log('Completato'),
  });

// Output:
// Errore rilevato: Si è verificato un errore
// Valore: Valore di fallback dopo l'errore
// Completato
```

## Diverse strategie di gestione degli errori

### 1. Catturare gli errori e fornire valori alternativi

Utilizzare l'operatore `catchError` per catturare gli errori e fornire valori alternativi o stream alternativi.

```ts
import { of, throwError } from 'rxjs';
import { catchError } from 'rxjs';

const source$ = throwError(() => new Error('Errore nel recupero dei dati'));

source$.pipe(
  catchError(error => {
    console.error('Si è verificato un errore:', error.message);
    // Restituisce dati alternativi
    return of({ isError: true, data: [], message: 'Visualizzazione dei dati predefiniti' });
  })
).subscribe(data => console.log('Risultato:', data));

// Output:
// Si è verificato un errore: Errore nel recupero dei dati
// Risultato: {isError: true, data: Array(0), message: 'Visualizzazione dei dati predefiniti'}
```

### 2. Riprovare se si verifica un errore

Usare l'operatore `retry` o `retryWhen` per riprovare lo stream se si verifica un errore.

```ts
import { interval, throwError, of } from 'rxjs';
import { mergeMap, retry, tap } from 'rxjs';

let attemptCount = 0;

interval(1000).pipe(
  mergeMap(val => {
    if (++attemptCount <= 2) {
      return throwError(() => new Error(`Errore #${attemptCount}`));
    }
    return of('Successo!');
  }),
  tap(() => console.log('Esecuzione:', attemptCount)),
  retry(2), // Riprova fino a 2 volte
).subscribe({
  next: value => console.log('Valore:', value),
  error: err => console.error('Errore finale:', err.message),
});

// Output:
// Esecuzione: 3
// Valore: Successo!
// Esecuzione: 4
// Valore: Successo!
// Esecuzione: 5
// ...
```

### 3. Riprovare con backoff esponenziale

Il backoff esponenziale, che aumenta gradualmente l'intervallo di riprova, è efficace ad esempio per le richieste di rete.

```ts
import { throwError, timer, of } from 'rxjs';
import { retryWhen, tap, concatMap, catchError } from 'rxjs';

function fetchWithRetry() {
  let retryCount = 0;

  return throwError(() => new Error('Errore di rete')).pipe(
    retryWhen((errors) =>
      errors.pipe(
        // Conta il numero di errori
        tap((error) => console.log('Si è verificato un errore:', error.message)),
        // Ritardo con backoff esponenziale
        concatMap(() => {
          retryCount++;
          const delayMs = Math.min(1000 * Math.pow(2, retryCount), 10000);
          console.log(`${retryCount}° tentativo dopo ${delayMs}ms`);
          return timer(delayMs);
        }),
        // Riprova fino a 5 volte
        tap(() => {
          if (retryCount >= 5) {
            throw new Error('È stato superato il numero massimo di tentativi');
          }
        })
      )
    ),
    // Fallback finale
    catchError((error) => {
      console.error('Tutti i tentativi sono falliti:', error.message);
      return of({
        error: true,
        message: 'Connessione fallita. Riprovare più tardi.',
      });
    })
  );
}

fetchWithRetry().subscribe({
  next: (result) => console.log('Risultato:', result),
  error: (err) => console.error('Errore non gestito:', err),
});

// Output:
// Si è verificato un errore: Errore di rete
// 1° tentativo dopo 2000ms
// Si è verificato un errore: Errore di rete
// 2° tentativo dopo 4000ms
// Si è verificato un errore: Errore di rete
// 3° tentativo dopo 8000ms
// Si è verificato un errore: Errore di rete
// 4° tentativo dopo 10000ms
// Si è verificato un errore: Errore di rete
// 5° tentativo dopo 10000ms
// Tutti i tentativi sono falliti: È stato superato il numero massimo di tentativi
// Risultato: {error: true, message: 'Connessione fallita. Riprovare più tardi.'}
```

### 4. Rilascio delle risorse in caso di errore

Usare l'operatore `finalize` per rilasciare le risorse quando uno stream termina **completato o per errore**.
Finalize è utile quando si vuole garantire che il processo di pulizia venga eseguito non solo in caso di errore, ma anche al completamento normale.

```ts
import { throwError } from 'rxjs';
import { catchError, finalize } from 'rxjs';

let isLoading = true;

throwError(() => new Error('Errore di elaborazione'))
  .pipe(
    catchError((error) => {
      console.error('Gestione errore:', error.message);
      return throwError(() => error); // Rilancia l'errore
    }),
    finalize(() => {
      isLoading = false;
      console.log('Ripristino dello stato di caricamento:', isLoading);
    })
  )
  .subscribe({
    next: (value) => console.log('Valore:', value),
    error: (err) => console.error('Errore finale:', err.message),
    complete: () => console.log('Completato'),
  });

// Output:
// Gestione errore: Errore di elaborazione
// Errore finale: Errore di elaborazione
// Ripristino dello stato di caricamento: false
```

## Pattern di gestione degli errori

### Gestione degli errori con controllo della visualizzazione degli elementi UI

```ts
import { of, throwError } from 'rxjs';
import { catchError, finalize, tap } from 'rxjs';

function fetchData(shouldFail = false) {
  // Mostra l'indicatore di caricamento
  showLoadingIndicator();

  // Recupera i dati (successo o errore)
  return (
    shouldFail
      ? throwError(() => new Error('Errore API'))
      : of({ name: 'dati', value: 42 })
  ).pipe(
    tap((data) => {
      // Elaborazione in caso di successo
      updateUI(data);
    }),
    catchError((error) => {
      // Aggiornamento dell'interfaccia utente in caso di errore
      showErrorMessage(error.message);
      // Restituisce dati vuoti o un valore predefinito
      return of({ name: 'default', value: 0 });
    }),
    finalize(() => {
      // Nasconde l'indicatore di caricamento indipendentemente dal successo o dall'errore
      hideLoadingIndicator();
    })
  );
}

// Funzioni di supporto per le operazioni dell'interfaccia utente
function showLoadingIndicator() {
  console.log('Visualizzazione caricamento');
}
function hideLoadingIndicator() {
  console.log('Nasconde caricamento');
}
function updateUI(data: { name: string; value: number }) {
  console.log('Aggiornamento UI:', data);
}
function showErrorMessage(message: any) {
  console.log('Visualizzazione errore:', message);
}

// Esempio di utilizzo
fetchData(true).subscribe();

// Output:
// Visualizzazione caricamento
// Visualizzazione errore: Errore API
// Nasconde caricamento
```

### Gestione di più fonti di errore

```ts
import { forkJoin, of, throwError } from 'rxjs';
import { catchError, map } from 'rxjs';

// Simulare più richieste API
function getUser() {
  return of({ id: 1, name: 'Mario Rossi' });
}

function getPosts() {
  return throwError(() => new Error('Errore nel recupero dei post'));
}

function getComments() {
  return throwError(() => new Error('Errore nel recupero dei commenti'));
}

// Ottenere tutti i dati, consentire errori parziali
forkJoin({
  user: getUser().pipe(
    catchError((error) => {
      console.error('Errore nel recupero utente:', error.message);
      return of(null); // Restituisce null in caso di errore
    })
  ),
  posts: getPosts().pipe(
    catchError((error) => {
      console.error('Errore nel recupero dei post:', error.message);
      return of([]); // Restituisce un array vuoto in caso di errore
    })
  ),
  comments: getComments().pipe(
    catchError((error) => {
      console.error('Errore nel recupero dei commenti:', error.message);
      return of([]); // Restituisce un array vuoto in caso di errore
    })
  ),
})
  .pipe(
    map((result) => ({
      ...result,
      // Aggiunge un flag per indicare se c'è stato un errore parziale
      hasErrors:
        !result.user ||
        result.posts.length === 0 ||
        result.comments.length === 0,
    }))
  )
  .subscribe((data) => {
    console.log('Risultato finale:', data);

    if (data.hasErrors) {
      console.log(
        'Il recupero di alcuni dati è fallito, ma i dati disponibili saranno visualizzati'
      );
    }
  });

// Output:
// Errore nel recupero dei post: Errore nel recupero dei post
// Errore nel recupero dei commenti: Errore nel recupero dei commenti
// Risultato finale: {user: {...}, posts: Array(0), comments: Array(0), hasErrors: true}
// Il recupero di alcuni dati è fallito, ma i dati disponibili saranno visualizzati
```

## Best practice per la gestione degli errori

1. **Catturare sempre gli errori**: aggiungere sempre la gestione degli errori nella catena degli Observable. Questo è particolarmente importante per gli stream di lunga durata.

2. **Fornire messaggi di errore significativi**: includere informazioni nell'oggetto errore per aiutare a determinare dove si è verificato e cosa lo ha causato.

3. **Rilasciare correttamente le risorse**: usare `finalize` per assicurarsi che le risorse siano rilasciate indipendentemente dal successo o dal fallimento.

4. **Considerare le strategie di retry**: specialmente per le operazioni di rete, l'implementazione di una corretta strategia di retry migliora l'affidabilità.

5. **Gestione degli errori user-friendly**: nell'interfaccia utente, fornire informazioni comprensibili agli utenti, piuttosto che visualizzare messaggi di errore tecnici così come sono.

```ts
// Esempio: conversione in messaggi di errore user-friendly
function getErrorMessage(error: any): string {
  if (error.status === 401) {
    return 'La sessione è scaduta. Si prega di effettuare nuovamente il login.';
  } else if (error.status === 404) {
    return 'La risorsa richiesta non è stata trovata.';
  } else if (error.status >= 500) {
    return 'Si è verificato un errore del server. Riprovare più tardi.';
  }
  return 'Si è verificato un errore inatteso.';
}
```

## Riepilogo

La gestione degli errori in RxJS è una parte importante per garantire la robustezza dell'applicazione. Utilizzando la giusta combinazione di operatori come `catchError`, `retry` e `finalize`, si possono gestire diversi scenari di errore. Progettare una strategia completa di gestione degli errori per migliorare l'esperienza dell'utente, piuttosto che limitarsi a catturare gli errori.

## 🔗 Sezioni correlate

- **[Errori comuni e come affrontarli](/it/guide/anti-patterns/common-mistakes#9-soppressione-degli-errori)** - Rivedere gli anti-pattern sulla gestione degli errori
- **[retry e catchError](/it/guide/error-handling/retry-catch)** - Spiega un utilizzo più dettagliato
