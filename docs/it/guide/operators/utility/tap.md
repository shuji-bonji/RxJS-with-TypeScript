---
description: L'operatore tap è un operatore utility che permette di eseguire effetti collaterali senza influenzare il valore dello stream. Ideale per il debugging con output log, controllo stati di caricamento, tracciamento analisi, monitoraggio errori e altre applicazioni dove viene eseguita elaborazione esterna osservando lo stream. Gli effetti collaterali possono essere gestiti in codice dichiarativo mantenendo la type safety di TypeScript.
---

# tap - Esegui Effetti Collaterali

L'operatore `tap` è usato per "eseguire effetti collaterali senza modificare lo stream."
Ideale per logging, debugging o altre operazioni che non influenzano i valori.

## 🔰 Sintassi e Operazione Base

Utilizzato in situazioni dove vuoi aggiungere solo effetti collaterali senza cambiare il flusso di valori.

```ts
import { of, tap } from 'rxjs';

of(42).pipe(
  tap(value => console.log('tap:', value))
).subscribe();
// Output:
// tap: 42
```

In questo esempio, il valore emesso da `of(42)` viene registrato nel log mentre passa attraverso `tap`.
Poiché `tap` passa il valore "così com'è", non ha effetto sul contenuto dello stream.

[🌐 Documentazione Ufficiale RxJS - tap](https://rxjs.dev/api/index/function/tap)

## 💡 Casi d'Uso Tipici

`tap` è spesso usato per i seguenti scopi:

- Debugging e logging
- Alternare stato di caricamento
- Mostrare notifiche toast
- Attivare aggiornamenti UI

```ts
import { of, tap, map } from 'rxjs';

of(Math.random()).pipe(
  tap(val => console.log('Valore recuperato:', val)),
  map(n => n > 0.5 ? 'Alto' : 'Basso'),
  tap(label => console.log('Etichetta:', label))
).subscribe();
// Output:
// Valore recuperato: 0.09909888881113504
// Etichetta: Basso
```


## 🧪 Esempio di Codice Pratico (con UI)

Di seguito un esempio di aggiunta di log al DOM usando tap.

```ts
import { of } from 'rxjs';
import { tap, map } from 'rxjs';

// Elemento per output log
const logOutput = document.createElement('div');
document.body.appendChild(logOutput);

// Sequenza di valori
of(1, 2, 3, 4, 5)
  .pipe(
    tap((val) => {
      console.log(`Valore originale: ${val}`);

      // Aggiungi log alla UI
      const logEntry = document.createElement('div');
      logEntry.textContent = `tap: Valore ${val} passato`;
      logEntry.style.color = '#666';
      logOutput.appendChild(logEntry);
    }),
    map((val) => val * 10),
    tap((val) => {
      console.log(`Valore trasformato: ${val}`);

      // Aggiungi log alla UI
      const logEntry = document.createElement('div');
      logEntry.textContent = `tap: Valore trasformato ${val}`;
      logEntry.style.color = '#090';
      logOutput.appendChild(logEntry);
    })
  )
  .subscribe((val) => {
    // Mostra risultato finale nella UI
    const resultItem = document.createElement('div');
    resultItem.textContent = `Risultato: ${val}`;
    resultItem.style.fontWeight = 'bold';
    logOutput.appendChild(resultItem);
  });

```


## ✅ Riepilogo

- `tap` è un operatore specializzato per **inserire effetti collaterali**
- **Output log e aggiornamenti UI** possono essere fatti senza cambiare il flusso di valori
- Può essere combinato con `finalize` e `catchError` per un controllo più pratico
