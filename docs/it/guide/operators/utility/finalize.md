---
description: finalize è un operatore utility RxJS che definisce un processo da eseguire ogni volta che un Observable viene completato, va in errore o viene disiscritto. È ideale per situazioni che richiedono pulizia alla fine dello stream, come rilascio risorse, fine visualizzazione caricamento e operazioni di pulizia. Garantisce che le operazioni vengano eseguite in modo affidabile come try-finally e aiuta a prevenire perdite di memoria.
---

# finalize - Elaborazione al Completamento

L'operatore `finalize` definisce un processo che viene chiamato ogni volta che l'**Observable viene completato, va in errore o viene disiscritto**.
È ideale per processi che "devono essere eseguiti" come pulizia e rilascio del caricamento UI.

## 🔰 Sintassi e Operazione Base

```ts
import { of } from 'rxjs';
import { finalize } from 'rxjs';

of('Completato')
  .pipe(finalize(() => console.log('Lo stream è terminato')))
  .subscribe(console.log);
// Output:
// Completato
// Lo stream è terminato
```

In questo esempio, il processo in `finalize` viene eseguito dopo aver emesso un valore in `of()`.
**Viene chiamato in modo affidabile sia per `complete` che per `error`**.

[🌐 Documentazione Ufficiale RxJS - finalize](https://rxjs.dev/api/index/function/finalize)

## 💡 Esempio di Utilizzo Tipico

Di seguito un esempio di alternanza della visualizzazione del caricamento prima e dopo lo streaming.

```ts
import { of } from 'rxjs';
import { tap, delay, finalize } from 'rxjs';

let isLoading = false;

of('Dati')
  .pipe(
    tap(() => {
      isLoading = true;
      console.log('Caricamento iniziato');
    }),
    delay(1000),
    finalize(() => {
      isLoading = false;
      console.log('Caricamento terminato');
    })
  )
  .subscribe((value) => console.log('Recuperato:', value));
// Output:
// Caricamento iniziato
// Recuperato: Dati
// Caricamento terminato
```

## 🧪 Esempio di Codice Pratico (con UI)

```ts
import { interval } from 'rxjs';
import { take, finalize, tap } from 'rxjs';

// Area di visualizzazione output
const finalizeOutput = document.createElement('div');
finalizeOutput.innerHTML = '<h3>Esempio finalize:</h3>';
document.body.appendChild(finalizeOutput);

// Indicatore di caricamento
const loadingIndicator = document.createElement('div');
loadingIndicator.textContent = 'Caricamento dati...';
loadingIndicator.style.backgroundColor = '#e0f7fa';
loadingIndicator.style.padding = '10px';
loadingIndicator.style.borderRadius = '5px';
finalizeOutput.appendChild(loadingIndicator);

// Visualizzazione progresso
const progressContainer = document.createElement('div');
progressContainer.style.marginTop = '10px';
finalizeOutput.appendChild(progressContainer);

// Elemento messaggio di completamento
const completionMessage = document.createElement('div');
completionMessage.style.marginTop = '10px';
completionMessage.style.fontWeight = 'bold';
finalizeOutput.appendChild(completionMessage);

// Simulazione recupero dati
interval(500)
  .pipe(
    take(5), // Recupera 5 valori
    tap((val) => {
      const progressItem = document.createElement('div');
      progressItem.textContent = `Elaborazione elemento ${val + 1}...`;
      progressContainer.appendChild(progressItem);
    }),
    finalize(() => {
      loadingIndicator.style.display = 'none';
      completionMessage.textContent = 'Elaborazione completata!';
      completionMessage.style.color = 'green';
    })
  )
  .subscribe({
    complete: () => {
      const successMsg = document.createElement('div');
      successMsg.textContent = 'Tutti i dati caricati con successo.';
      completionMessage.appendChild(successMsg);
    },
  });
```

## ✅ Riepilogo

- `finalize` viene **sempre eseguito** indipendentemente da completamento, errore o terminazione manuale
- Ideale per processi di pulizia e terminazione caricamento
- Può essere combinato con altri operatori (`tap`, `delay`, ecc.) per una **pulizia asincrona sicura**
