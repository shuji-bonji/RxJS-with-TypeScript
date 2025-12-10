---
description: "Descrive robuste strategie di gestione degli errori che combinano gli operatori retry e catchError. Imparerete con esempi pratici di codice come implementare la sicurezza di tipo in TypeScript, compresi i retry per i fallimenti temporanei, i modelli di backoff esponenziali, i retry condizionali e la corretta gestione dei fallback."
---

# retry e catchError - Combinazioni efficaci per la gestione degli errori

Approfondimento sui due operatori fondamentali per la gestione degli errori in RxJS: `retry` e `catchError`. La loro combinazione può fornire una solida strategia di gestione degli errori.

## retry - Riprovare in caso di errore (pattern di base)

L'operatore `retry` è usato per **riavviare l'esecuzione di uno stream un numero specificato di volte** quando si verifica un errore nello stream. È particolarmente utile per le operazioni che possono temporaneamente fallire, come le richieste di rete.

[🌐 Documentazione ufficiale RxJS - retry](https://rxjs.dev/api/index/function/retry)

### Pattern di base

```ts
import { Observable, of } from 'rxjs';
import { retry, map } from 'rxjs';

// Funzione che genera errori casuali
function getDataWithRandomError(): Observable<string> {
  return of('dati').pipe(
    map(() => {
      if (Math.random() < 0.7) {
        throw new Error('Si è verificato un errore casuale');
      }
      return 'Acquisizione dati riuscita!';
    })
  );
}

// Riprova fino a 3 volte
getDataWithRandomError()
  .pipe(retry(3))
  .subscribe({
    next: (data) => console.log('Successo:', data),
    error: (err) => console.error('Errore (dopo 3 tentativi):', err.message),
  });

// Output:
// Successo: Acquisizione dati riuscita!
// Errore (dopo 3 tentativi): Si è verificato un errore casuale ⇦ Visualizzato dopo 3 fallimenti
```

### Monitoraggio in tempo reale dello stato dei tentativi

```ts
import { Observable, of } from 'rxjs';
import { retry, tap, catchError, map } from 'rxjs';

let attempts = 0;

function simulateFlakyRequest(): Observable<string> {
  return of('richiesta').pipe(
    tap(() => {
      attempts++;
      console.log(`Tentativo #${attempts}`);
    }),
    map(() => {
      if (attempts < 3) {
        throw new Error(`Errore #${attempts}`);
      }
      return 'Successo!';
    })
  );
}

simulateFlakyRequest()
  .pipe(
    retry(3),
    catchError((error) => {
      console.log('Tutti i tentativi falliti:', error.message);
      return of('Valore di fallback');
    })
  )
  .subscribe({
    next: (result) => console.log('Risultato finale:', result),
    complete: () => console.log('Completato'),
  });

// Output:
// Tentativo #1
// Tentativo #2
// Tentativo #3
// Risultato finale: Successo!
// Completato
```

> [!NOTE] Tempistica dei retry e Scheduler
> Quando si specifica un tempo di ritardo con l'operatore `retry` (ad esempio `retry({ delay: 1000 })`), viene utilizzato internamente **asyncScheduler**. Utilizzando lo scheduler, è possibile controllare con precisione la tempistica dei tentativi e utilizzare il tempo virtuale durante i test.
>
> Per ulteriori informazioni, vedere [Tipi di scheduler e loro utilizzo - Controllo dei retry degli errori](/it/guide/schedulers/types#controllo-dei-retry-degli-errori).

## catchError - Cattura degli errori e gestione alternativa (pattern di base)

L'operatore `catchError` cattura gli errori che si verificano in uno stream e li gestisce **restituendo un Observable alternativo**. Ciò consente di continuare l'elaborazione senza interrompere lo stream quando si verifica un errore.

[🌐 Documentazione ufficiale RxJS - catchError](https://rxjs.dev/api/index/function/catchError)

### Pattern di base

```ts
import { of, throwError } from 'rxjs';
import { catchError } from 'rxjs';

throwError(() => new Error('Errore chiamata API')) // RxJS 7 o successivo, si raccomanda il formato funzione
  .pipe(
    catchError((error) => {
      console.error('Si è verificato un errore:', error.message);
      return of('Valore predefinito quando si verifica un errore');
    })
  )
  .subscribe({
    next: (value) => console.log('Valore:', value),
    complete: () => console.log('Completato'),
  });

// Output:
// Si è verificato un errore: Errore chiamata API
// Valore: Valore predefinito quando si verifica un errore
// Completato
```

### Rilancio dell'errore

Se si vuole lanciare nuovamente l'errore dopo che è stato registrato:

```ts
import { throwError } from 'rxjs';
import { catchError } from 'rxjs';

throwError(() => new Error('Errore originale')) // RxJS 7 o successivo, si raccomanda il formato funzione
  .pipe(
    catchError((error) => {
      console.error('Registrazione errore:', error.message);
      // Rilancia l'errore
      return throwError(() => new Error('Errore convertito'));
    })
  )
  .subscribe({
    next: (value) => console.log('Valore:', value),
    error: (err) => console.error('Errore finale:', err.message),
    complete: () => console.log('Completato'),
  });

// Output:
// Registrazione errore: Errore originale
// Errore finale: Errore convertito
```

## Combinazione di retry e catchError

Nelle applicazioni reali, è comune utilizzare una combinazione di `retry` e `catchError`. Questa combinazione consente di risolvere gli errori temporanei con un nuovo tentativo, fornendo al contempo un valore di fallback in caso di fallimento definitivo.

```ts
import { of, throwError } from 'rxjs';
import { retry, catchError, tap } from 'rxjs';

function fetchData() {
  // Observable che lancia un errore
  return throwError(() => new Error('Errore di rete')) // RxJS 7 o successivo, si raccomanda il formato funzione
    .pipe(
    // Per il debug
    tap(() => console.log('Tentativo di acquisizione dati')),
    // Riprova fino a 3 volte
    retry(3),
    // Se tutti i tentativi falliscono
    catchError((error) => {
      console.error('Tutti i tentativi falliti:', error.message);
      // Restituisce il valore predefinito
      return of({
        error: true,
        data: null,
        message: 'Acquisizione dati fallita',
      });
    })
  );
}

fetchData().subscribe({
  next: (result) => console.log('Risultato:', result),
  complete: () => console.log('Elaborazione completata'),
});

// Output:
// Tutti i tentativi falliti: Errore di rete
// Risultato: {error: true, data: null, message: 'Acquisizione dati fallita'}
// Elaborazione completata
```

## Strategia di retry avanzata: retryWhen

Se si ha bisogno di una strategia di retry più flessibile, si può usare l'operatore `retryWhen`. Questo consente di personalizzare la tempistica e la logica dei tentativi.

[🌐 Documentazione ufficiale RxJS - retryWhen](https://rxjs.dev/api/index/function/retryWhen)

### Retry con backoff esponenziale

Il pattern di backoff esponenziale (intervalli di retry gradualmente crescenti) è comune per riprovare le richieste di rete. Questo riduce il carico sul server in attesa della risoluzione di problemi temporanei.

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
          // timer utilizza internamente asyncScheduler
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
      console.error('Tutti i tentativi falliti:', error.message);
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
```

> [!TIP] Controllo dettagliato dei retry utilizzando lo scheduler
> L'esempio precedente usa `timer()`, ma se si ha bisogno di un controllo più sofisticato, si può specificare esplicitamente uno scheduler per regolare con precisione la tempistica dei tentativi o usare il tempo virtuale durante i test.
>
> Per ulteriori informazioni, vedere [Tipi di scheduler e loro utilizzo - Controllo dei retry degli errori](/it/guide/schedulers/types#controllo-dei-retry-degli-errori).

## Debug dei retry

Quando si esegue il debug del processo di retry, è importante tenere traccia del numero di tentativi e del risultato di ogni tentativo. Di seguito troverete modi pratici per monitorare lo stato dei tentativi in tempo reale.

### Metodo 1: callback error di tap (base)

Il callback `error` dell'operatore `tap` può essere usato per contare il numero di tentativi quando si verifica un errore.

```typescript
import { throwError, of, retry, catchError, tap } from 'rxjs';
let attemptCount = 0;

throwError(() => new Error('Errore temporaneo'))
  .pipe(
    tap({
      error: () => {
        attemptCount++;
        console.log(`Numero di tentativi: ${attemptCount}`);
      }
    }),
    retry(2),
    catchError((error) => {
      console.log(`Tentativi finali: ${attemptCount}`);
      return of(`Errore finale: ${error.message}`);
    })
  )
  .subscribe({
    next: console.log,
    error: err => console.error('Errore di sottoscrizione:', err)
  });

// Output:
// Numero di tentativi: 1
// Numero di tentativi: 2
// Numero di tentativi: 3
// Tentativi finali: 3
// Errore finale: Errore temporaneo
```

> [!NOTE] Limitazioni con throwError
> `throwError` non emette un valore e lancia immediatamente un errore, quindi il callback `next` di `tap` non viene eseguito. È necessario utilizzare il callback `error`.

### Metodo 2: Tracciamento dettagliato con retryWhen (consigliato)

Per tracciare informazioni più dettagliate (numero di tentativi, tempo di ritardo, dettagli sugli errori), utilizzare `retryWhen`.

```typescript
import { throwError, of, timer, retryWhen, mergeMap, catchError } from 'rxjs';
throwError(() => new Error('Errore temporaneo'))
  .pipe(
    retryWhen((errors) =>
      errors.pipe(
        mergeMap((error, index) => {
          const retryCount = index + 1;
          console.log(`━━━━━━━━━━━━━━━━━━━━━━━━━━━`);
          console.log(`🔄 Retry ${retryCount}° tentativo`);
          console.log(`   Errore: ${error.message}`);

          if (retryCount > 2) {
            console.log(`❌ Raggiunto il numero massimo di tentativi`);
            console.log(`━━━━━━━━━━━━━━━━━━━━━━━━━━━`);
            throw error;
          }

          const delayMs = 1000;
          console.log(`⏳ Riprova dopo ${delayMs}ms...`);
          console.log(`━━━━━━━━━━━━━━━━━━━━━━━━━━━`);

          return timer(delayMs);
        })
      )
    ),
    catchError((error) => {
      console.log(`\nRisultato finale: tutti i retry falliti`);
      return of(`Errore finale: ${error.message}`);
    })
  )
  .subscribe(result => console.log('Risultato:', result));

// Output:
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━
// 🔄 Retry 1° tentativo
//    Errore: Errore temporaneo
// ⏳ Riprova dopo 1000ms...
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━
// (attende 1 secondo)
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━
// 🔄 Retry 2° tentativo
//    Errore: Errore temporaneo
// ⏳ Riprova dopo 1000ms...
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━
// (attende 1 secondo)
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━
// 🔄 Retry 3° tentativo
//    Errore: Errore temporaneo
// ❌ Raggiunto il numero massimo di tentativi
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━
//
// Risultato finale: tutti i retry falliti
// Risultato: Errore finale: Errore temporaneo
```

### Metodo 3: Tracciare i tentativi con un Observable personalizzato

Per gli Observable che emettono valori, come le richieste API effettive, è possibile tenere traccia del numero di tentativi con un Observable personalizzato.

```typescript
import { Observable, of, retry, catchError } from 'rxjs';
let attemptCount = 0;

// Observable in grado di contare il numero di tentativi
const retryableStream$ = new Observable(subscriber => {
  attemptCount++;
  console.log(`[Tentativo ${attemptCount}°]`);

  // I primi due tentativi falliscono, il terzo ha successo
  if (attemptCount < 3) {
    subscriber.error(new Error(`Fallimento (tentativo ${attemptCount})`));
  } else {
    subscriber.next('Dati di successo');
    subscriber.complete();
  }
});

retryableStream$
  .pipe(
    retry(2),
    catchError((error) => {
      console.log(`[Completato] Totale ${attemptCount} tentativi falliti`);
      return of(`Errore finale: ${error.message}`);
    })
  )
  .subscribe({
    next: data => console.log('[Risultato]', data),
    complete: () => console.log('[Completato]')
  });

// Output:
// [Tentativo 1°]
// [Tentativo 2°]
// [Tentativo 3°]
// [Risultato] Dati di successo
// [Completato]
```

### Metodo 4: Backoff esponenziale e logging

Schema di log dettagliato con richieste API pratiche.

```typescript
import { timer, throwError, of, retryWhen, mergeMap, catchError, finalize } from 'rxjs';
import { ajax } from 'rxjs/ajax';

function fetchWithRetryLogging(url: string, maxRetries = 3) {
  let startTime = Date.now();

  return ajax.getJSON(url).pipe(
    retryWhen((errors) =>
      errors.pipe(
        mergeMap((error, index) => {
          const retryCount = index + 1;
          const elapsed = Date.now() - startTime;

          console.log(`━━━━━━━━━━━━━━━━━━━━━━━━━━━`);
          console.log(`🔄 Informazioni retry`);
          console.log(`   Tentativi: ${retryCount}/${maxRetries}`);
          console.log(`   Errore: ${error.message || error.status}`);
          console.log(`   Tempo trascorso: ${elapsed}ms`);

          if (retryCount >= maxRetries) {
            console.log(`❌ Raggiunto il numero massimo di tentativi`);
            console.log(`━━━━━━━━━━━━━━━━━━━━━━━━━━━`);
            throw error;
          }

          // Backoff esponenziale
          const delayMs = Math.min(1000 * Math.pow(2, index), 10000);
          console.log(`⏳ Riprova dopo ${delayMs}ms...`);
          console.log(`━━━━━━━━━━━━━━━━━━━━━━━━━━━`);

          return timer(delayMs);
        })
      )
    ),
    catchError((error) => {
      const totalTime = Date.now() - startTime;
      console.log(`\n❌ Fallimento finale (Tempo totale: ${totalTime}ms)`);
      return of({ error: true, message: 'Acquisizione dati fallita' });
    }),
    finalize(() => {
      const totalTime = Date.now() - startTime;
      console.log(`\n✅ Elaborazione completata (Tempo totale: ${totalTime}ms)`);
    })
  );
}

// Esempio di utilizzo
fetchWithRetryLogging('https://jsonplaceholder.typicode.com/users/1').subscribe({
  next: data => console.log('Dati:', data),
  error: err => console.error('Errore:', err)
});
```

### Metodo 5: Oggetto di configurazione retry in RxJS 7.4+

In RxJS 7.4 e successivi, è possibile passare un oggetto di configurazione a `retry`.

```typescript
import { throwError, of, retry, catchError, tap } from 'rxjs';
let attemptCount = 0;

throwError(() => new Error('Errore temporaneo'))
  .pipe(
    tap({
      subscribe: () => {
        attemptCount++;
        console.log(`Tentativo ${attemptCount}°`);
      },
      error: (err) => console.log(`Errore verificato:`, err.message)
    }),
    retry({
      count: 2,
      delay: 1000, // Attende 1 secondo e riprova (usa internamente asyncScheduler)
      resetOnSuccess: true
    }),
    catchError((error) => {
      console.log(`Fallimento finale (totale ${attemptCount} tentativi)`);
      return of(`Errore finale: ${error.message}`);
    })
  )
  .subscribe(result => console.log('Risultato:', result));

// Output:
// Tentativo 1°
// Errore verificato: Errore temporaneo
// Tentativo 2°
// Errore verificato: Errore temporaneo
// Tentativo 3°
// Errore verificato: Errore temporaneo
// Fallimento finale (totale 3 tentativi)
// Risultato: Errore finale: Errore temporaneo
```

> [!TIP] Approccio consigliato per il debug dei retry
> - **Sviluppo**: il metodo 2 (retryWhen) o il metodo 4 (log dettagliato) sono ottimali
> - **In produzione**: basato sul metodo 4, con l'aggiunta dell'invio dei log a un servizio di monitoraggio degli errori
> - **Caso semplice**: il metodo 1 (error di tap) o il metodo 5 (configurazione retry) sono sufficienti
>
> **Informazioni correlate**:
> - Per il controllo della tempistica dei retry vedere [Tipi di scheduler e loro utilizzo - Controllo dei retry degli errori](/it/guide/schedulers/types#controllo-dei-retry-degli-errori)
> - Per una panoramica sulle tecniche di debug, vedere [Tecniche di debug RxJS - Tracciare i tentativi di retry](/it/guide/debugging/#scenario-6-tracciare-il-numero-di-tentativi-di-retry)

## Esempio di utilizzo in un'applicazione reale: Richiesta API

Esempio di utilizzo di questi operatori in una richiesta API reale.

```ts
import { Observable, of } from 'rxjs';
import { ajax } from 'rxjs/ajax';
import { retry, catchError, finalize, tap } from 'rxjs';

// Stato di caricamento
let isLoading = false;

function fetchUserData(userId: string): Observable<any> {
  isLoading = true;

  return ajax.getJSON(`https://jsonplaceholder.typicode.com/users/${userId}`).pipe(
    // Debug della richiesta
    tap((response) => console.log('Risposta API:', response)),
    // Riprova fino a 2 volte per gli errori di rete
    retry(2),
    // Gestione degli errori
    catchError((error) => {
      if (error.status === 404) {
        return of({ error: true, message: 'Utente non trovato' });
      } else if (error.status >= 500) {
        return of({ error: true, message: 'Si è verificato un errore del server' });
      }
      return of({ error: true, message: 'Si è verificato un errore sconosciuto' });
    }),
    // Viene sempre eseguito, indipendentemente dal successo o dal fallimento
    finalize(() => {
      isLoading = false;
      console.log('Caricamento completato');
    })
  );
}

// Esempio di utilizzo
fetchUserData('123').subscribe({
  next: (data) => {
    if (data.error) {
      // Mostra le informazioni sull'errore
      console.error('Errore:', data.message);
    } else {
      // Visualizzazione dei dati
      console.log('Dati utente:', data);
    }
  },
});

// Output:
// GET https://jsonplaceholder.typicode.com/users/123 net::ERR_NAME_NOT_RESOLVED
// GET https://jsonplaceholder.typicode.com/users/123 net::ERR_NAME_NOT_RESOLVED
// Si è verificato un errore sconosciuto
// Caricamento completato
// GET https://jsonplaceholder.typicode.com/users/123 net::ERR_NAME_NOT_RESOLVED
```

## Best Practice

### Quando si dovrebbe usare retry

- Quando si prevedono **errori temporanei** (ad es. problemi di connessione di rete)
- **Problemi temporanei** sul lato server (ad es. carico elevato, timeout)
- Per errori che possono essere **risolti** con un nuovo tentativo

### Quando non si dovrebbe usare retry

- **Errori di autenticazione** (401, 403) - il retry non li risolverà
- **Risorsa inesistente** (404) - non viene trovata dopo il retry
- **Errore di validazione** (400) - c'è un problema con la richiesta stessa
- **Errore di programmazione lato client** - il retry è inutile

### Uso efficace di catchError

- **Elaborazione diversa** a seconda del **tipo** di errore
- Fornire **messaggi chiari** all'utente
- Restituire **dati di fallback** se appropriato
- **Convertire gli errori** come richiesto

## Riepilogo

La combinazione di `retry` e `catchError` fornisce una solida gestione degli errori. Gli errori temporanei possono essere recuperati con un nuovo tentativo, mentre gli errori persistenti possono essere gestiti appropriatamente con il fallback per migliorare l'esperienza dell'utente. Nelle applicazioni reali, è importante selezionare la strategia appropriata e fornire un meccanismo di fallback a seconda della natura dell'errore.

Nella prossima sezione descriveremo l'operatore `finalize` per il rilascio delle risorse e il processo di completamento dello stream.
