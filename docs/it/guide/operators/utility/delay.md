---
description: L'operatore delay ritarda il timing di emissione di ogni valore nell'Observable di una quantità di tempo specificata, rendendolo efficace per la direzione UI e il controllo asincrono.
---

# delay - Ritardo dei Valori

L'operatore `delay` è usato per ritardare l'emissione di ogni valore in uno stream di una quantità di tempo specificata.
È utile per sincronizzare animazioni e regolare il timing della visualizzazione del feedback all'utente.


## 🔰 Sintassi e Operazione Base

Questa è la configurazione minima per emettere un valore dopo un certo tempo.

```ts
import { of } from 'rxjs';
import { delay } from 'rxjs';

of('Ciao')
  .pipe(
    delay(1000) // Emetti valore dopo 1 secondo
  )
  .subscribe(console.log);
// Output:
// Ciao
```

In questo esempio, il valore creato da `of('Ciao')` viene ricevuto da `subscribe()` con un ritardo di 1 secondo.

[🌐 Documentazione Ufficiale RxJS - delay](https://rxjs.dev/api/index/function/delay)

## 💡 Esempio di Utilizzo Tipico

Questo è un esempio di utilizzo di delay per regolare il timing dell'emissione in una situazione dove vengono emessi più valori.

```ts
import { of } from 'rxjs';
import { delay, concatMap } from 'rxjs';

of('A', 'B', 'C')
  .pipe(
    concatMap(
      (val, index) => of(val).pipe(delay(1000 * index)) // A immediatamente, B dopo 1 secondo, C dopo 2 secondi
    )
  )
  .subscribe(console.log);
// Output:
// A
// B
// C
```

In questo modo, è anche possibile impostare un ritardo separato per ogni valore combinandolo con `concatMap`.


## 🧪 Esempio di Codice Pratico (con UI)

```ts
import { of } from 'rxjs';
import { delay, tap } from 'rxjs';

// Area di visualizzazione output
const delayOutput = document.createElement('div');
delayOutput.innerHTML = '<h3>Esempio delay:</h3>';
document.body.appendChild(delayOutput);

// Funzione per visualizzare l'ora corrente
function addTimeLog(message: string) {
  const now = new Date();
  const time =
    now.toLocaleTimeString('it-IT', { hour12: false }) +
    '.' +
    now.getMilliseconds().toString().padStart(3, '0');

  const logItem = document.createElement('div');
  logItem.textContent = `${time}: ${message}`;
  delayOutput.appendChild(logItem);
}

// Registra ora di inizio
addTimeLog('Inizio');

// Sequenza di valori
of('A', 'B', 'C')
  .pipe(
    tap((val) => addTimeLog(`Prima che il valore ${val} venga emesso`)),
    delay(1000), // Ritardo di 1 secondo
    tap((val) => addTimeLog(`Valore ${val} emesso dopo 1 secondo`))
  )
  .subscribe();
```


## ✅ Riepilogo

- `delay` è un operatore per **controllare il timing dell'output dell'Observable**
- Può essere combinato con `concatMap` per **controllare il ritardo per valore**
- Utile per **regolazioni asincrone** per migliorare la UX, come output alla UI e direzione del timer
