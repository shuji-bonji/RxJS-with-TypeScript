---
description: materialize è un operatore utility RxJS che converte le notifiche Observable (next, error, complete) in oggetti Notification. È ideale per situazioni dove vuoi manipolare la notifica stessa, come gestire errori come dati, debugging e logging delle notifiche, registrazione di meta-informazioni, ecc. dematerialize permette di ripristinare il formato originale e l'elaborazione delle notifiche type-safe con l'inferenza dei tipi TypeScript.
---

# materialize - Oggettifica le Notifiche

L'operatore `materialize` converte le **notifiche Observable (next, error, complete) in oggetti Notification**. Questo permette di gestire non solo i valori ma anche errori e completamenti come dati.

## 🔰 Sintassi e Operazione Base

Converte uno stream normale in uno stream di oggetti Notification.

```ts
import { of } from 'rxjs';
import { materialize } from 'rxjs';

of(1, 2, 3)
  .pipe(materialize())
  .subscribe(notification => {
    console.log(notification);
  });
// Output:
// Notification { kind: 'N', value: 1, error: undefined, hasValue: true }
// Notification { kind: 'N', value: 2, error: undefined, hasValue: true }
// Notification { kind: 'N', value: 3, error: undefined, hasValue: true }
// Notification { kind: 'C', value: undefined, error: undefined, hasValue: false }
```

La proprietà `kind` dell'oggetto Notification:
- `'N'`: next (valore emesso)
- `'E'`: error
- `'C'`: complete

[🌐 Documentazione Ufficiale RxJS - materialize](https://rxjs.dev/api/index/function/materialize)

## 💡 Esempi di Utilizzo Tipici

- **Datificazione errori**: Tratta gli errori come parte dello stream
- **Debugging e logging**: Tracciamento dettagliato delle notifiche
- **Registrazione meta-informazioni**: Registra quando e che tipo di notifiche si verificano
- **Combinazione di stream con errori**: Gestisci errori in più stream in modo unificato

## 🧪 Esempio di Codice Pratico 1: Tratta Errori come Dati

Questo esempio mostra come trattare errori che normalmente interromperebbero uno stream come dati e continuare.

```ts
import { of, throwError, concat } from 'rxjs';
import { materialize, map } from 'rxjs';

// Creazione UI
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'materialize - Datificazione errori';
container.appendChild(title);

const output = document.createElement('div');
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
container.appendChild(output);

function addLog(message: string, color: string) {
  const logItem = document.createElement('div');
  logItem.style.padding = '5px';
  logItem.style.marginBottom = '3px';
  logItem.style.backgroundColor = color;
  logItem.textContent = message;
  output.appendChild(logItem);
}

// Gestione errori normale (stream interrotto)
addLog('--- Gestione errori normale ---', '#e3f2fd');
concat(
  of(1, 2),
  throwError(() => new Error('Errore verificato')),
  of(3, 4)  // Non eseguito qui
).subscribe({
  next: v => addLog(`Valore: ${v}`, '#c8e6c9'),
  error: err => addLog(`❌ Errore: ${err.message}`, '#ffcdd2'),
  complete: () => addLog('Completato', '#e3f2fd')
});

// Usando materialize (stream continua)
setTimeout(() => {
  addLog('--- Usando materialize ---', '#e3f2fd');

  concat(
    of(1, 2),
    throwError(() => new Error('Errore verificato')),
    of(3, 4)
  )
    .pipe(
      materialize(),
      map(notification => {
        if (notification.kind === 'N') {
          return `Valore: ${notification.value}`;
        } else if (notification.kind === 'E') {
          return `Errore (datificato): ${notification.error?.message}`;
        } else {
          return 'Completato';
        }
      })
    )
    .subscribe({
      next: msg => {
        const color = msg.includes('Errore') ? '#fff9c4' : '#c8e6c9';
        addLog(msg, color);
      },
      complete: () => addLog('Stream completato', '#e3f2fd')
    });
}, 1000);
```

- Gli errori normali interrompono lo stream
- Con `materialize`, gli errori vengono trattati come dati e lo stream continua

## 🧪 Esempio di Codice Pratico 2: Debug Logging

Ecco un esempio che logga in dettaglio tutte le notifiche.

```ts
import { interval, throwError, of } from 'rxjs';
import { materialize, take, mergeMap } from 'rxjs';

// Creazione UI
const container2 = document.createElement('div');
container2.style.marginTop = '20px';
document.body.appendChild(container2);

const title2 = document.createElement('h3');
title2.textContent = 'materialize - Debug logging';
container2.appendChild(title2);

const output2 = document.createElement('div');
output2.style.border = '1px solid #ccc';
output2.style.padding = '10px';
output2.style.maxHeight = '250px';
output2.style.overflow = 'auto';
output2.style.fontFamily = 'monospace';
output2.style.fontSize = '12px';
container2.appendChild(output2);

function addLog2(message: string) {
  const now = new Date();
  const timestamp = now.toLocaleTimeString('it-IT', { hour12: false }) +
    '.' + now.getMilliseconds().toString().padStart(3, '0');

  const logItem = document.createElement('div');
  logItem.style.marginBottom = '2px';
  logItem.textContent = `[${timestamp}] ${message}`;
  output2.appendChild(logItem);
}

interval(500)
  .pipe(
    take(5),
    mergeMap(value => {
      // Genera errore quando il valore è 3
      if (value === 3) {
        return throwError(() => new Error('Errore al valore 3'));
      }
      return of(value);
    }),
    materialize()
  )
  .subscribe({
    next: notification => {
      switch (notification.kind) {
        case 'N':
          addLog2(`[NEXT] valore: ${notification.value}`);
          break;
        case 'E':
          addLog2(`[ERROR] ${notification.error?.message}`);
          break;
        case 'C':
          addLog2('[COMPLETE]');
          break;
      }
    },
    complete: () => {
      addLog2('--- Observer completato ---');
    }
  });
```

- Logging uniforme di tutti i tipi di notifica (next, error, complete)
- Traccia l'ordine in cui le notifiche si verificano con timestamp
- Utile per debugging e monitoraggio

## 🆚 Confronto con Stream Normali

```ts
import { of } from 'rxjs';
import { materialize } from 'rxjs';

// Stream normale
of(1, 2, 3).subscribe({
  next: v => console.log('Valore:', v),
  complete: () => console.log('Completato')
});
// Output:
// Valore: 1
// Valore: 2
// Valore: 3
// Completato

// Usando materialize
of(1, 2, 3)
  .pipe(materialize())
  .subscribe({
    next: n => console.log('Notifica:', n),
    complete: () => console.log('Completato')
  });
// Output:
// Notifica: Notification { kind: 'N', value: 1, ... }
// Notifica: Notification { kind: 'N', value: 2, ... }
// Notifica: Notification { kind: 'N', value: 3, ... }
// Notifica: Notification { kind: 'C', ... }
// Completato
```

## Manipola Oggetto Notification

```ts
import { of } from 'rxjs';
import { materialize, map } from 'rxjs';

of(10, 20, 30)
  .pipe(
    materialize(),
    map(notification => {
      // Proprietà dell'oggetto Notification
      return {
        kind: notification.kind,           // 'N', 'E', 'C'
        hasValue: notification.hasValue,   // Ha valore
        value: notification.value,         // Valore (per next)
        error: notification.error          // Errore (per error)
      };
    })
  )
  .subscribe(console.log);
// Output:
// { kind: 'N', hasValue: true, value: 10, error: undefined }
// { kind: 'N', hasValue: true, value: 20, error: undefined }
// { kind: 'N', hasValue: true, value: 30, error: undefined }
// { kind: 'C', hasValue: false, value: undefined, error: undefined }
```

## ⚠️ Note Importanti

### 1. Gli Errori Non Interrompono lo Stream

Quando usi `materialize`, gli errori vengono trattati come dati e lo stream non viene interrotto.

```ts
import { of, throwError, concat } from 'rxjs';
import { materialize } from 'rxjs';

concat(
  of(1),
  throwError(() => new Error('Errore')),
  of(2)
)
  .pipe(materialize())
  .subscribe({
    next: n => console.log('Notifica:', n.kind),
    error: () => console.log('Gestore errori'),  // Non chiamato
    complete: () => console.log('Completato')
  });
// Output:
// Notifica: N
// Notifica: E  ← Anche gli errori vengono trattati come next
// Completato
```

### 2. Combinazione con dematerialize

Gli stream trasformati con `materialize` possono essere ripristinati con `dematerialize`.

```ts
import { of } from 'rxjs';
import { materialize, dematerialize } from 'rxjs';

of(1, 2, 3)
  .pipe(
    materialize(),
    // Qualche elaborazione qui
    dematerialize()  // Ripristina
  )
  .subscribe(console.log);
// Output: 1, 2, 3
```

### 3. Impatto sulle Performance

C'è un overhead nella generazione degli oggetti Notification. Usa solo quando necessario in ambiente di produzione.

## 📚 Operatori Correlati

- **[dematerialize](./dematerialize)** - Ripristina l'oggetto Notification a notifica normale
- **[tap](./tap)** - Esegui un effetto collaterale (per scopi di debugging)
- **[catchError](/it/guide/error-handling/retry-catch)** - Gestione errori

## ✅ Riepilogo

L'operatore `materialize` converte una notifica in un oggetto Notification.

- ✅ Può gestire errori come dati
- ✅ Utile per debugging e logging
- ✅ Può registrare meta-informazioni sulle notifiche
- ✅ Può essere annullato con `dematerialize`
- ⚠️ Gli errori non interromperanno più lo stream
- ⚠️ Nota l'overhead delle performance
