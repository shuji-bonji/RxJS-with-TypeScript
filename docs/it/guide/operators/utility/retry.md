---
description: L'operatore retry risottoscrive e ritenta la sorgente un numero specificato di volte quando si verifica un errore in Observable. È utile per recuperare da guasti temporanei di comunicazione, come errori di rete, o per processi che potrebbero avere successo se ritentati dopo un fallimento.
---

# retry - Ritenta in Caso di Errore

L'operatore `retry` è un operatore che **risottoscrive l'Observable sorgente un numero specificato di volte** quando si verifica un errore.
È adatto per **processi che potrebbero avere successo se ritentati dopo un fallimento**, come guasti temporanei di rete.

## 🔰 Sintassi e Operazione Base

### retry(count) - Forma Base

```ts
import { throwError, of } from 'rxjs';
import { retry, catchError } from 'rxjs';

throwError(() => new Error('Errore temporaneo'))
  .pipe(
    retry(2), // Ritenta fino a 2 volte
    catchError((error) => of(`Errore finale: ${error.message}`))
  )
  .subscribe(console.log);
// Output:
// Errore finale: Errore temporaneo
```

In questo esempio, vengono effettuati fino a due tentativi dopo il primo fallimento, e un messaggio viene emesso in fallback se tutti falliscono.

### retry(config) - Formato Oggetto di Configurazione (RxJS 7.4+)

In RxJS 7.4 e successivi, è possibile un controllo più dettagliato passando un oggetto di configurazione.

```ts
import { throwError, of } from 'rxjs';
import { retry, catchError, tap } from 'rxjs';

let attemptCount = 0;

throwError(() => new Error('Errore temporaneo'))
  .pipe(
    tap({
      subscribe: () => {
        attemptCount++;
        console.log(`Tentativo ${attemptCount}`);
      }
    }),
    retry({
      count: 2,           // Ritenta fino a 2 volte
      delay: 1000,        // Attendi 1 secondo prima di ritentare (usa asyncScheduler internamente)
      resetOnSuccess: true // Resetta il conteggio in caso di successo
    }),
    catchError((error) => of(`Errore finale: ${error.message}`))
  )
  .subscribe(console.log);

// Output:
// Tentativo 1
// Tentativo 2
// Tentativo 3
// Errore finale: Errore temporaneo
```

> [!NOTE] Controllo Timing dei Tentativi
> Quando l'opzione `delay` è specificata, viene usato internamente **asyncScheduler**. Per un controllo più dettagliato del timing dei tentativi (exponential backoff, ecc.), vedi [Tipi di Scheduler e Utilizzo - Controllo Retry Errori](/it/guide/schedulers/types#error-retry-control).

[🌐 Documentazione Ufficiale RxJS - retry](https://rxjs.dev/api/index/function/retry)

## 💡 Esempio di Utilizzo Tipico

L'esempio seguente è una configurazione che ritenta **elaborazione asincrona con successo/fallimento casuale** fino a 3 volte.

```ts
import { interval, throwError, of } from 'rxjs';
import { mergeMap, retry, catchError } from 'rxjs';

let attempt = 0;

interval(1000)
  .pipe(
    mergeMap(() => {
      attempt++;
      const shouldFail = Math.random() < 0.8;

      if (shouldFail) {
        return throwError(() => new Error(`Fallimento #${attempt}`));
      } else {
        return of(`Successo #${attempt}`);
      }
    }),
    retry(3),
    catchError((err) => of(`Fallimento finale: ${err.message}`))
  )
  .subscribe(console.log);
// Output:
// Successo #1
// Successo #5
// Successo #6
// Fallimento finale: Fallimento #7
```

## 🧪 Esempio di Codice Pratico (con UI)

```ts
import { interval, throwError, of } from 'rxjs';
import { mergeMap, retry, catchError } from 'rxjs';

// Area di visualizzazione output
const retryOutput = document.createElement('div');
retryOutput.innerHTML = '<h3>Esempio retry (Simulazione Richiesta API):</h3>';
document.body.appendChild(retryOutput);

// Visualizzazione stato richiesta
const requestStatus = document.createElement('div');
requestStatus.style.marginTop = '10px';
requestStatus.style.padding = '10px';
requestStatus.style.border = '1px solid #ddd';
requestStatus.style.maxHeight = '200px';
requestStatus.style.overflowY = 'auto';
retryOutput.appendChild(requestStatus);

// Richiesta API che riesce o fallisce casualmente
let attemptCount = 0;

function simulateRequest() {
  attemptCount++;

  const logEntry = document.createElement('div');
  logEntry.textContent = `Tentativo #${attemptCount} Invio richiesta...`;
  requestStatus.appendChild(logEntry);

  return interval(1000).pipe(
    mergeMap(() => {
      const shouldFail = Math.random() < 0.8;

      if (shouldFail) {
        const errorMsg = document.createElement('div');
        errorMsg.textContent = `Tentativo #${attemptCount} Fallito: Errore di rete`;
        errorMsg.style.color = 'red';
        requestStatus.appendChild(errorMsg);

        return throwError(() => new Error('Errore di rete'));
      } else {
        const successMsg = document.createElement('div');
        successMsg.textContent = `Tentativo #${attemptCount} Successo!`;
        successMsg.style.color = 'green';
        requestStatus.appendChild(successMsg);

        return of({ id: 1, name: 'Dati recuperati con successo' });
      }
    }),
    retry(3),
    catchError((err) => {
      const finalError = document.createElement('div');
      finalError.textContent = `Tutti i tentativi falliti: ${err.message}`;
      finalError.style.color = 'red';
      finalError.style.fontWeight = 'bold';
      requestStatus.appendChild(finalError);

      return of({ error: true, message: 'Retry fallito' });
    })
  );
}

// Bottone avvio richiesta
const startButton = document.createElement('button');
startButton.textContent = 'Avvia Richiesta';
startButton.style.padding = '8px 16px';
startButton.style.marginTop = '10px';
retryOutput.insertBefore(startButton, requestStatus);

startButton.addEventListener('click', () => {
  attemptCount = 0;
  requestStatus.innerHTML = '';
  startButton.disabled = true;

  simulateRequest().subscribe((result) => {
    const resultElement = document.createElement('div');
    if ('error' in result) {
      resultElement.textContent = `Risultato finale: ${result.message}`;
      resultElement.style.backgroundColor = '#ffebee';
    } else {
      resultElement.textContent = `Risultato finale: ${result.name}`;
      resultElement.style.backgroundColor = '#e8f5e9';
    }

    resultElement.style.padding = '10px';
    resultElement.style.marginTop = '10px';
    resultElement.style.borderRadius = '5px';
    requestStatus.appendChild(resultElement);

    startButton.disabled = false;
  });
});
```

## ✅ Riepilogo

- `retry(n)` ritenta fino a `n` volte se l'Observable fallisce
- `retry` viene **ritentato fino a completamento con successo** (fallimenti continui risultano in un errore)
- Utile per **API asincrone e richieste di rete** dove si verificano fallimenti temporanei
- Comunemente combinato con `catchError` per specificare **elaborazione di fallback**
- Da RxJS 7.4+, è possibile specificare `delay`, `resetOnSuccess`, ecc. in formato oggetto di configurazione

## Pagine Correlate

- [retry e catchError](/it/guide/error-handling/retry-catch) - Pattern per combinare retry e catchError, esempi di utilizzo pratico
- [Debugging Retry](/it/guide/error-handling/retry-catch#retry-debugging) - Come tracciare il conteggio tentativi (5 pattern di implementazione)
- [Tipi di Scheduler e Utilizzo](/it/guide/schedulers/types#error-retry-control) - Controllo dettagliato timing retry, implementazione exponential backoff
- [Tecniche di Debugging RxJS](/it/guide/debugging/#scenario-6-track-retry-attempt-count) - Scenari di debugging retry
