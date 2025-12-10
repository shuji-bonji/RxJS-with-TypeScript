---
description: dematerialize è un operatore utility RxJS che ripristina gli oggetti Notification alle notifiche normali (next, error, complete) e esegue la trasformazione inversa di materialize. È ideale per ripristinare le notifiche dopo l'elaborazione, filtrare o convertire errori, riordinare o bufferizzare notifiche, o qualsiasi altra situazione dove vuoi elaborare le notifiche come dati e poi restituirle al loro formato originale.
---

# dematerialize - Ripristina Oggetto Notification

L'operatore `dematerialize` **converte** un oggetto Notification in una notifica normale (next, error, complete). Esegue la trasformazione inversa di `materialize`, ripristinando la notifica datificata alla sua forma originale.

## 🔰 Sintassi e Operazione Base

Converte uno stream di oggetti Notification in uno stream normale.

```ts
import { of } from 'rxjs';
import { materialize, dematerialize } from 'rxjs';

of(1, 2, 3)
  .pipe(
    materialize(),     // Converti in oggetto Notification
    dematerialize()    // Ripristina
  )
  .subscribe({
    next: v => console.log('Valore:', v),
    complete: () => console.log('Completato')
  });
// Output:
// Valore: 1
// Valore: 2
// Valore: 3
// Completato
```

[🌐 Documentazione Ufficiale RxJS - dematerialize](https://rxjs.dev/api/index/function/dematerialize)

## 💡 Esempi di Utilizzo Tipici

- **Ripristina notifiche dopo elaborazione**: Ripristinale al loro formato originale dopo l'elaborazione con materialize
- **Filtraggio errori**: Escludi solo certi errori
- **Riordinamento dell'ordine delle notifiche**: Ripristina dopo aver ordinato le notifiche come dati
- **Ripristino dopo debugging**: Ripristina operazione normale dopo logging, ecc.

## 🧪 Esempio di Codice Pratico 1: Filtraggio Selettivo degli Errori

Questo è un esempio di esclusione solo di certi errori ed elaborazione del resto come normale.

```ts
import { of, throwError, concat } from 'rxjs';
import { materialize, dematerialize, filter } from 'rxjs';

// Creazione UI
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'dematerialize - Filtraggio errori';
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

// Stream con errori
const source$ = concat(
  of(1, 2),
  throwError(() => new Error('Errore ignorabile')),
  of(3, 4),
  throwError(() => new Error('Errore critico')),
  of(5)
);

source$
  .pipe(
    materialize(),
    filter(notification => {
      // Filtra solo "Errore ignorabile"
      if (notification.kind === 'E') {
        const errorMessage = notification.error?.message || '';
        if (errorMessage.includes('Ignorabile')) {
          addLog(`🔇 Ignorato: ${errorMessage}`, '#fff9c4');
          return false;  // Escludi questo errore
        }
      }
      return true;
    }),
    dematerialize()  // Ripristina al formato originale
  )
  .subscribe({
    next: v => addLog(`✅ Valore: ${v}`, '#c8e6c9'),
    error: err => addLog(`❌ Errore: ${err.message}`, '#ffcdd2'),
    complete: () => addLog('Completato', '#e3f2fd')
  });
```

- Gli "Errori ignorabili" vengono esclusi e lo stream continua
- Gli "Errori critici" vengono passati al gestore errori come al solito
- Gestione selettiva degli errori possibile

## 🧪 Esempio di Codice Pratico 2: Notifica Ritardata

Questo è un esempio di bufferizzazione temporanea di una notifica e poi ripristino.

```ts
import { from, interval, take, delay } from 'rxjs';
import { materialize, dematerialize, bufferTime, concatMap } from 'rxjs';

// Creazione UI
const container2 = document.createElement('div');
container2.style.marginTop = '20px';
document.body.appendChild(container2);

const title2 = document.createElement('h3');
title2.textContent = 'dematerialize - Buffering e ritardo';
container2.appendChild(title2);

const output2 = document.createElement('div');
output2.style.border = '1px solid #ccc';
output2.style.padding = '10px';
output2.style.maxHeight = '200px';
output2.style.overflow = 'auto';
container2.appendChild(output2);

function addLog2(message: string) {
  const now = new Date();
  const timestamp = now.toLocaleTimeString('it-IT', { hour12: false }) +
    '.' + now.getMilliseconds().toString().padStart(3, '0');

  const logItem = document.createElement('div');
  logItem.textContent = `[${timestamp}] ${message}`;
  output2.appendChild(logItem);
}

addLog2('Inizio - emetti valori ogni secondo, elabora in batch ogni 2 secondi');

interval(1000)
  .pipe(
    take(6),
    materialize(),
    bufferTime(2000),      // Bufferizza ogni 2 secondi
    concatMap(notifications => {
      addLog2(`--- Elaborazione ${notifications.length} notifiche dal buffer ---`);
      return from(notifications).pipe(
        delay(500),        // Ritarda ogni notifica di 0.5 secondi
        dematerialize()    // Ripristina al formato originale
      );
    })
  )
  .subscribe({
    next: v => addLog2(`Valore: ${v}`),
    complete: () => addLog2('Completato')
  });
```

- Bufferizza le notifiche ogni 2 secondi
- Recupera dal buffer e ritarda l'elaborazione
- Ripristina come stream originale con `dematerialize`

## 🆚 Relazione con materialize

```ts
import { of } from 'rxjs';
import { materialize, dematerialize, map } from 'rxjs';

of(1, 2, 3)
  .pipe(
    materialize(),           // Converti in Notification
    map(notification => {
      // Elabora come oggetto Notification
      console.log('kind:', notification.kind);
      return notification;
    }),
    dematerialize()          // Ripristina
  )
  .subscribe(v => console.log('Valore:', v));
// Output:
// kind: N
// Valore: 1
// kind: N
// Valore: 2
// kind: N
// Valore: 3
// kind: C
```

| Flusso di Processo | Descrizione |
|:---|:---|
| Stream originale | Valore normale (next), errore (error), completamento (complete) |
| ↓ `materialize()` | Stream di oggetto Notification |
| Elaborazione intermedia | Elaborazione e filtraggio come Notification |
| ↓ `dematerialize()` | Ripristina a stream normale |
| Stream finale | Valore normale, errore, complete |

## ⚠️ Note Importanti

### 1. Le Notifiche Errore Vengono Convertite in Errori Reali

```ts
import { of, throwError, concat } from 'rxjs';
import { materialize, dematerialize } from 'rxjs';

// Converti ogni Observable in oggetto notification con materialize()
concat(
  of(1).pipe(materialize()),
  throwError(() => new Error('Errore')).pipe(materialize()),
  of(2).pipe(materialize())  // Non eseguito dopo errore
)
  .pipe(
    dematerialize()
  )
  .subscribe({
    next: v => console.log('Valore:', v),
    error: err => console.log('Errore:', err.message)
  });
// Output:
// Valore: 1
// Errore: Errore
```

Quando viene raggiunta una notifica di errore, lo stream viene interrotto con un errore.

### 2. La Notifica di Completamento Completa lo Stream

```ts
import { of, EMPTY, concat } from 'rxjs';
import { materialize, dematerialize } from 'rxjs';

// Converti ogni Observable in oggetto notification con materialize()
concat(
  of(1).pipe(materialize()),
  of(2).pipe(materialize()),
  EMPTY.pipe(materialize()),  // Notifica di completamento
  of(3).pipe(materialize())   // Non eseguito dopo completamento
)
  .pipe(
    dematerialize()
  )
  .subscribe({
    next: v => console.log('Valore:', v),
    complete: () => console.log('Completato')
  });
// Output:
// Valore: 1
// Valore: 2
// Completato
```

Nessun valore viene emesso dopo la notifica di completamento.

### 3. Oggetto Notification Non Valido

`dematerialize` si aspetta un oggetto Notification corretto.

```ts
import { of } from 'rxjs';
import { dematerialize } from 'rxjs';

// ❌ Passare valori normali a dematerialize causa errore
of(1, 2, 3)
  .pipe(
    dematerialize()  // Non è un oggetto Notification
  )
  .subscribe({
    next: console.log,
    error: err => console.error('Errore:', err.message)
  });
// Si verifica errore
```

## Esempi di Combinazione Pratica

```ts
import { interval, throwError, of, concat } from 'rxjs';
import { materialize, dematerialize, take, mergeMap, map } from 'rxjs';

// Esempio di conversione errori in warning
interval(500)
  .pipe(
    take(10),
    mergeMap(value => {
      // Genera errore solo quando è 5
      if (value === 5) {
        return throwError(() => new Error(`Errore al valore ${value}`));
      }
      return of(value);
    }),
    materialize(),
    map(notification => {
      // Converti errori in messaggi di warning
      if (notification.kind === 'E') {
        console.warn('Warning:', notification.error?.message);
        // Emetti valore speciale invece di errore (generato da materialize())
        return { kind: 'N' as const, value: -1 };
      }
      return notification;
    }),
    dematerialize()
  )
  .subscribe({
    next: v => console.log('Valore:', v),
    error: err => console.error('Errore:', err),  // Non chiamato
    complete: () => console.log('Completato')
  });
// Output:
// Valore: 0, 1, 2, 3, 4
// Warning: Errore al valore 5
// Valore: -1  (invece di errore)
// Valore: 6, 7, 8, 9
// Completato
```

## 📚 Operatori Correlati

- **[materialize](./materialize)** - Converti notifica in oggetto Notification
- **[catchError](/it/guide/error-handling/retry-catch)** - Gestione errori
- **[retry](./retry)** - Ritenta in caso di errore

## ✅ Riepilogo

L'operatore `dematerialize` restituisce l'oggetto Notification a una notifica normale.

- ✅ Conversione inversa di `materialize`
- ✅ Ripristina la notifica al suo formato originale dopo l'elaborazione
- ✅ Permette filtraggio e conversione degli errori
- ✅ Può essere usato per riordinare o bufferizzare notifiche
- ⚠️ Le notifiche errore agiscono come errori reali
- ⚠️ La notifica di completamento completa lo stream
- ⚠️ Richiede oggetto Notification corretto
