---
description: "Descrive una strategia completa di gestione degli errori per RxJS, includendo come combinare gli operatori catchError, retry, retryWhen e finalize, retry con backoff esponenziale, classificazione degli errori e gestione appropriata, gestori di errori globali e altre robuste strategie di gestione degli errori in TypeScript. Come implementare una gestione robusta degli errori in TypeScript."
---

# Strategia di gestione degli errori di RxJS

La gestione degli errori in RxJS è un aspetto importante della programmazione reattiva. L'implementazione di una corretta gestione degli errori migliora la robustezza e l'affidabilità dell'applicazione. Questo documento descrive le diverse strategie di gestione degli errori disponibili in RxJS.

## Modelli di base

RxJS gestisce gli errori come parte del ciclo di vita di Observable. La gestione degli errori di base comprende i seguenti metodi.

```ts
import { of, throwError } from 'rxjs';
import { catchError } from 'rxjs';

// Errore.Observable
const error$ = throwError(() => new Error('Si è verificato un errore.')); // RxJS 7Formato della funzione consigliato d'ora in poi

// Gestione degli errori di base
error$
  .pipe(
    catchError((error: unknown) => {
      const message = error instanceof Error ? error.message : String(error);
      console.error('Cattura dell'errore:', message);
      return of('Valore di fallback dopo l'errore');
    })
  )
  .subscribe({
    next: (value) => console.log('Valore:', value),
    error: (err) => console.error('Errori non gestiti:', err),
    complete: () => console.log('Completato'),
  });

// Uscita:
// Cattura dell'errore: Si è verificato un errore.
// Valore: Valore di fallback dopo l'errore
// Completato
```

## Varie strategie di gestione degli errori

### 1. catturare gli errori e fornire valori alternativi

Usare l'operatore catchError per catturare gli errori e fornire valori alternativi o flussi alternativi.

```ts
import { of, throwError } from 'rxjs';
import { catchError } from 'rxjs';

const source$ = throwError(() => new Error('Errore nell'acquisizione dei dati'));

source$.pipe(
  catchError((error: unknown) => {
    const message = error instanceof Error ? error.message : String(error);
    console.error('Si è verificato un errore:', message);
    // Restituzione di dati alternativi
    return of({ isError: true, data: [], message: 'Visualizzazione dei dati predefiniti' });
  })
).subscribe(data => console.log('Risultato:', data));

// Uscita:
// Si è verificato un errore: Errore nell'acquisizione dei dati
// Risultato: {isError: true, data: Array(0), message: 'Visualizzazione dei dati predefiniti'}
```

### 2. Riprovare se si verifica un errore

Utilizzare l'operatore `retry` per riprovare il flusso in caso di errore (a partire dalla versione 7.3, si raccomanda il formato `retry({ count, delay })` che sostituisce il vecchio `retryWhen`).

```ts
import { interval, throwError, of } from 'rxjs';
import { mergeMap, retry, tap } from 'rxjs';

let attemptCount = 0;

interval(1000).pipe(
  mergeMap(val => {
    if (++attemptCount <= 2) {
      return throwError(() => new Error(`Errore #${attemptCount}`));
    }
    return of('Successo！');
  }),
  tap(() => console.log('Esecuzione:', attemptCount)),
  retry(2), // Max.2Tentativi
).subscribe({
  next: value => console.log('Valore:', value),
  error: err => console.error('Errore finale:', err.message),
});

// Uscita:
// Esecuzione: 3
// Valore: Successo！
// Esecuzione: 4
// Valore: Successo！
// Esecuzione: 5
// ...
```

### 3. Riprova con backoff esponenziale

Il backoff esponenziale, che aumenta gradualmente l'intervallo di riprova, è efficace, ad esempio per le richieste di rete.

```ts
import { throwError, timer, of } from 'rxjs';
import { retry, tap, catchError } from 'rxjs';

function fetchWithRetry() {
  return throwError(() => new Error('Errore di rete')).pipe(
    // RxJS 7.3+ Consigliato: retry({ count, delay }) Formato
    retry({
      count: 5, // Max.5Fino a tre tentativi
      delay: (error: unknown, retryCount) => {
        const message = error instanceof Error ? error.message : String(error);
        console.log('Si è verificato un errore:', message);
        // Arretramento esponenziale (max.10secondi)
        const delayMs = Math.min(1000 * Math.pow(2, retryCount), 10000);
        console.log(`${retryCount}Riprova una seconda volta${delayMs}msEseguito dopo`);
        return timer(delayMs);
      }
    }),
    // Ricaduta finale
    catchError((error: unknown) => {
      const message = error instanceof Error ? error.message : String(error);
      console.error('Tutti i tentativi sono falliti:', message);
      return of({
        error: true,
        message: 'Connessione fallita. Riprovare più tardi.',
      });
    })
  );
}

fetchWithRetry().subscribe({
  next: (result) => console.log('Risultato:', result),
  error: (err) => console.error('Errori non gestiti:', err),
});

// Uscita:
// Si è verificato un errore: Errore di rete
// 1Riprova una seconda volta2000msEseguito dopo
// Si è verificato un errore: Errore di rete
// 2Riprova una seconda volta4000msEseguito dopo
// Si è verificato un errore: Errore di rete
// 3Riprova una seconda volta8000msEseguito dopo
// Si è verificato un errore: Errore di rete
// 4Riprova una seconda volta10000msEseguito dopo
// Si è verificato un errore: Errore di rete
// 5Riprova una seconda volta10000msEseguito dopo
// Tutti i tentativi sono falliti: Numero massimo di tentativi superato
// Risultato: {error: true, message: 'Connessione fallita. Riprovare più tardi.'}
```

### 4. Rilascio di risorse su errore

Usare l'operatore finalize per rilasciare le risorse quando un flusso termina **completo o per errore**.
Finalize è utile quando si vuole garantire che il processo di pulizia venga eseguito non solo in caso di errore, ma anche al completamento normale.

```ts
import { throwError } from 'rxjs';
import { catchError, finalize } from 'rxjs';

let isLoading = true;

throwError(() => new Error('Errore di elaborazione'))
  .pipe(
    catchError((error: unknown) => {
      const message = error instanceof Error ? error.message : String(error);
      console.error('Errore di elaborazione:', message);
      return throwError(() => error); // Rilasciare l'errore
    }),
    finalize(() => {
      isLoading = false;
      console.log('Ripristinare lo stato di caricamento:', isLoading);
    })
  )
  .subscribe({
    next: (value) => console.log('Valore:', value),
    error: (err) => console.error('Errore finale:', err.message),
    complete: () => console.log('Completato'),
  });

// Uscita:
// Errore di elaborazione: Errore di elaborazione
// Errore finale: Errore di elaborazione
// Ripristinare lo stato di caricamento: false
```

## Modello di gestione degli errori

### Gestione degli errori che include il controllo della visualizzazione degli elementi dell'IU

```ts
import { of, throwError } from 'rxjs';
import { catchError, finalize, tap } from 'rxjs';

function fetchData(shouldFail = false) {
  // Visualizzazione del caricamento
  showLoadingIndicator();

  // Acquisizione dei dati (successo o errore)
  return (
    shouldFail
      ? throwError(() => new Error('APIErrore'))
      : of({ name: 'Dati', value: 42 })
  ).pipe(
    tap((data) => {
      // In caso di successo
      updateUI(data);
    }),
    catchError((error: unknown) => {
      const message = error instanceof Error ? error.message : String(error);
      // In caso di erroreUIAggiornamento
      showErrorMessage(message);
      // Restituzione di dati vuoti o di un valore predefinito
      return of({ name: 'Predefinito', value: 0 });
    }),
    finalize(() => {
      // Disattiva la visualizzazione del caricamento indipendentemente dal successo o dall'errore
      hideLoadingIndicator();
    })
  );
}

// UIFunzioni di aiuto per le operazioni
function showLoadingIndicator() {
  console.log('Visualizzazione del caricamento');
}
function hideLoadingIndicator() {
  console.log('Nascondere il caricamento');
}
function updateUI(data: { name: string; value: number }) {
  console.log('UIAggiornamento:', data);
}
function showErrorMessage(message: any) {
  console.log('Visualizzazione degli errori:', message);
}

// Esempio di utilizzo
fetchData(true).subscribe();

// Uscita:
// Visualizzazione del caricamento
// Visualizzazione degli errori: APIErrore
// Nascondere il caricamento
```

### Gestione di più fonti di errore

{```ts
import { forkJoin, of, throwError } from 'rxjs';
import { catchError, map } from 'rxjs';

// Simulazione multiplaAPISimulazione di richieste
function getUser() {
  return of({ id: 1, name: 'Taro Yamada' });
}

function getPosts() {
  return throwError(() => new Error('Errore di post acquisizione'));
}

function getComments() {
  return throwError(() => new Error('Commento dell'errore di acquisizione'));
}

// Recuperare tutti i dati e consentire errori parziali
forkJoin({
  user: getUser().pipe(
    catchError((error: unknown) => {
      const message = error instanceof Error ? error.message : String(error);
      console.error('Errore di acquisizione dell'utente:', message);
      return of(null); // In caso di errorenullRestituisce un array vuoto
    })
  ),
  posts: getPosts().pipe(
    catchError((error: unknown) => {
      const message = error instanceof Error ? error.message : String(error);
      console.error('Errore di post acquisizione:', message);
      return of([]); // In caso di errore viene restituito un array vuoto
    })
  ),
  comments: getComments().pipe(
    catchError((error: unknown) => {
      const message = error instanceof Error ? error.message : String(error);
      console.error('Commento dell'errore di acquisizione:', message);
      return of([]); // In caso di errore viene restituito un array vuoto
    })
  ),
})
  .pipe(
    map((result) => ({
      ...result,
      // Aggiungere un flag per indicare se c'è stato un errore parziale
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
        'Errore nel recupero di alcuni dati, ma mostra i dati disponibili'
      );
    }
  });

// Uscita:
// Errore di post acquisizione: Errore di post acquisizione
// Commento dell'errore di acquisizione: Commento dell'errore di acquisizione
// Risultato finale: {user: {…}, posts: Array(0), comments: Array(0), hasErrors: true}
// Errore nel recupero di alcuni dati, ma mostra i dati disponibili
```

## Migliori pratiche per la gestione degli errori

1. **Cogliere sempre gli errori**: aggiungere sempre la gestione degli errori nella catena di Observable. Questo è particolarmente importante per i flussi di lunga durata.

2.**Fornire messaggi di errore significativi**: includere informazioni nell'oggetto errore per aiutare a determinare dove si è verificato e cosa lo ha causato.

3, **rilasciare correttamente le risorse**: usare finalize per assicurarsi che le risorse siano rilasciate indipendentemente dal successo o dal fallimento.

4.**Considerare le strategie di retry**: specialmente per le operazioni di rete, l'implementazione di una corretta strategia di retry migliora l'affidabilità.

5. **Gestione degli errori user-friendly**: nell'interfaccia utente, fornire informazioni comprensibili agli utenti, piuttosto che visualizzare messaggi di errore tecnici come sono.

```

```ts
// Esempio.：Conversione in messaggi di errore di facile comprensione
function getErrorMessage(error: any): string {
  if (error.status === 401) {
    return 'La sessione è scaduta. Effettuare nuovamente il login.';
  } else if (error.status === 404) {
    return 'La risorsa richiesta non è stata trovata.';
  } else if (error.status >= 500) {
    return 'Si è verificato un errore del server. Riprovare più tardi.';
  }
  return 'Si è verificato un errore imprevisto.';
}
```

## Riepilogo.

La gestione degli errori in RxJS è una parte importante per garantire la robustezza dell'applicazione. Utilizzando la giusta combinazione di operatori come catchError, retry e finalize, si possono gestire diversi scenari di errore. Progettare una strategia completa di gestione degli errori per migliorare l'esperienza dell'utente, piuttosto che limitarsi a catturare gli errori.

## 🔗 Sezioni correlate.

- **[Errori comuni e come affrontarli](/it/guide/anti-patterns/common-mistakes#9-error-grasping)** - Rivedere gli anti-patterns sulla gestione degli errori.
- **[retry e catchError](/it/guide/error-handling/retry-catch)** - Spiega un utilizzo più dettagliato