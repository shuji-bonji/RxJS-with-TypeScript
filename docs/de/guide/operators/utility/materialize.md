---
description: materialize ist ein RxJS Utility-Operator, der Observable-Benachrichtigungen (next, error, complete) in Notification-Objekte umwandelt. Er eignet sich optimal für Szenarien, in denen Benachrichtigungen selbst manipuliert werden sollen, wie Behandlung von Fehlern als Daten, Debugging und Logging von Benachrichtigungen oder Aufzeichnung von Meta-Informationen. Mit dematerialize kann zum Originalformat zurückgewandelt werden, und durch TypeScript-Typinferenz ist eine typsichere Benachrichtigungsverarbeitung möglich.
---

# materialize - Objektivierung von Benachrichtigungen

Der `materialize`-Operator **wandelt Observable-Benachrichtigungen (next, error, complete) in Notification-Objekte** um. Dadurch können nicht nur Werte, sondern auch Fehler und Abschlüsse als Daten behandelt werden.

## 🔰 Grundlegende Syntax und Funktionsweise

Wandelt einen normalen Stream in einen Stream von Notification-Objekten um.

```ts
import { of } from 'rxjs';
import { materialize } from 'rxjs';

of(1, 2, 3)
  .pipe(materialize())
  .subscribe(notification => {
    console.log(notification);
  });
// Ausgabe:
// Notification { kind: 'N', value: 1, error: undefined, hasValue: true }
// Notification { kind: 'N', value: 2, error: undefined, hasValue: true }
// Notification { kind: 'N', value: 3, error: undefined, hasValue: true }
// Notification { kind: 'C', value: undefined, error: undefined, hasValue: false }
```

`kind`-Property des Notification-Objekts:
- `'N'`: next (Werteemission)
- `'E'`: error (Fehler)
- `'C'`: complete (Abschluss)

[🌐 RxJS Offizielle Dokumentation - materialize](https://rxjs.dev/api/index/function/materialize)

## 💡 Typische Anwendungsfälle

- **Datifizierung von Fehlern**: Fehler als Teil des Streams behandeln
- **Debugging und Logging**: Detaillierte Verfolgung von Benachrichtigungen
- **Aufzeichnung von Meta-Informationen**: Aufzeichnen, wann und welche Benachrichtigungen aufgetreten sind
- **Zusammenführung von Streams mit Fehlern**: Einheitliche Verarbeitung von Fehlern mehrerer Streams

## 🧪 Praktisches Codebeispiel 1: Fehler als Daten behandeln

Beispiel für fortgesetzte Behandlung von Fehlern, die normalerweise den Stream unterbrechen würden, als Daten.

```ts
import { of, throwError, concat } from 'rxjs';
import { materialize, map } from 'rxjs';

// UI erstellen
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'materialize - Datifizierung von Fehlern';
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

// Normales Fehlerhandling (Stream-Unterbrechung)
addLog('--- Normales Fehlerhandling ---', '#e3f2fd');
concat(
  of(1, 2),
  throwError(() => new Error('Fehler aufgetreten')),
  of(3, 4)  // Wird hier nicht ausgeführt
).subscribe({
  next: v => addLog(`Wert: ${v}`, '#c8e6c9'),
  error: err => addLog(`❌ Fehler: ${err.message}`, '#ffcdd2'),
  complete: () => addLog('Abgeschlossen', '#e3f2fd')
});

// Verwendung von materialize (Stream-Fortsetzung)
setTimeout(() => {
  addLog('--- Verwendung von materialize ---', '#e3f2fd');

  concat(
    of(1, 2),
    throwError(() => new Error('Fehler aufgetreten')),
    of(3, 4)
  )
    .pipe(
      materialize(),
      map(notification => {
        if (notification.kind === 'N') {
          return `Wert: ${notification.value}`;
        } else if (notification.kind === 'E') {
          return `Fehler (datifiziert): ${notification.error?.message}`;
        } else {
          return 'Abgeschlossen';
        }
      })
    )
    .subscribe({
      next: msg => {
        const color = msg.includes('Fehler') ? '#fff9c4' : '#c8e6c9';
        addLog(msg, color);
      },
      complete: () => addLog('Stream abgeschlossen', '#e3f2fd')
    });
}, 1000);
```

- Normale Fehler unterbrechen den Stream
- Mit `materialize` werden Fehler als Daten behandelt und der Stream wird fortgesetzt

## 🧪 Praktisches Codebeispiel 2: Debug-Logging

Beispiel für detaillierte Log-Ausgabe aller Benachrichtigungen.

```ts
import { interval, throwError, of } from 'rxjs';
import { materialize, take, mergeMap } from 'rxjs';

// UI erstellen
const container2 = document.createElement('div');
container2.style.marginTop = '20px';
document.body.appendChild(container2);

const title2 = document.createElement('h3');
title2.textContent = 'materialize - Debug-Logging';
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
  const timestamp = now.toLocaleTimeString('de-DE', { hour12: false }) +
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
      // Bei Wert 3 Fehler auslösen
      if (value === 3) {
        return throwError(() => new Error('Fehler bei Wert 3'));
      }
      return of(value);
    }),
    materialize()
  )
  .subscribe({
    next: notification => {
      switch (notification.kind) {
        case 'N':
          addLog2(`[NEXT] value: ${notification.value}`);
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
      addLog2('--- Observer abgeschlossen ---');
    }
  });
```

- Einheitliche Log-Ausgabe aller Benachrichtigungstypen (next, error, complete)
- Verfolgung der Auftretensreihenfolge von Benachrichtigungen mit Zeitstempel
- Nützlich für Debugging und Monitoring

## 🆚 Vergleich mit normalem Stream

```ts
import { of } from 'rxjs';
import { materialize } from 'rxjs';

// Normaler Stream
of(1, 2, 3).subscribe({
  next: v => console.log('Wert:', v),
  complete: () => console.log('Abgeschlossen')
});
// Ausgabe:
// Wert: 1
// Wert: 2
// Wert: 3
// Abgeschlossen

// Verwendung von materialize
of(1, 2, 3)
  .pipe(materialize())
  .subscribe({
    next: n => console.log('Benachrichtigung:', n),
    complete: () => console.log('Abgeschlossen')
  });
// Ausgabe:
// Benachrichtigung: Notification { kind: 'N', value: 1, ... }
// Benachrichtigung: Notification { kind: 'N', value: 2, ... }
// Benachrichtigung: Notification { kind: 'N', value: 3, ... }
// Benachrichtigung: Notification { kind: 'C', ... }
// Abgeschlossen
```

## Manipulation von Notification-Objekten

```ts
import { of } from 'rxjs';
import { materialize, map } from 'rxjs';

of(10, 20, 30)
  .pipe(
    materialize(),
    map(notification => {
      // Properties des Notification-Objekts
      return {
        kind: notification.kind,           // 'N', 'E', 'C'
        hasValue: notification.hasValue,   // Hat Wert
        value: notification.value,         // Wert (bei next)
        error: notification.error          // Fehler (bei error)
      };
    })
  )
  .subscribe(console.log);
// Ausgabe:
// { kind: 'N', hasValue: true, value: 10, error: undefined }
// { kind: 'N', hasValue: true, value: 20, error: undefined }
// { kind: 'N', hasValue: true, value: 30, error: undefined }
// { kind: 'C', hasValue: false, value: undefined, error: undefined }
```

## ⚠️ Wichtige Hinweise

### 1. Fehler unterbrechen Stream nicht

Bei Verwendung von `materialize` werden Fehler als Daten behandelt und der Stream wird nicht unterbrochen.

```ts
import { of, throwError, concat } from 'rxjs';
import { materialize } from 'rxjs';

concat(
  of(1),
  throwError(() => new Error('Fehler')),
  of(2)
)
  .pipe(materialize())
  .subscribe({
    next: n => console.log('Benachrichtigung:', n.kind),
    error: () => console.log('Fehlerhandler'),  // Wird nicht aufgerufen
    complete: () => console.log('Abgeschlossen')
  });
// Ausgabe:
// Benachrichtigung: N
// Benachrichtigung: E  ← Fehler wird als next behandelt
// Abgeschlossen
```

### 2. Kombination mit dematerialize

Mit `materialize` umgewandelte Streams können mit `dematerialize` zurückgewandelt werden.

```ts
import { of } from 'rxjs';
import { materialize, dematerialize } from 'rxjs';

of(1, 2, 3)
  .pipe(
    materialize(),
    // Hier irgendeine Verarbeitung
    dematerialize()  // Zurückwandeln
  )
  .subscribe(console.log);
// Ausgabe: 1, 2, 3
```

### 3. Auswirkung auf Leistung

Erzeugung von Notification-Objekten verursacht Overhead. In der Produktionsumgebung nur bei Bedarf verwenden.

## 📚 Verwandte Operatoren

- **[dematerialize](./dematerialize)** - Zurückwandeln von Notification-Objekten in normale Benachrichtigungen
- **[tap](./tap)** - Ausführung von Nebenwirkungen (für Debug-Zwecke)
- **[catchError](../../error-handling/retry-catch)** - Fehlerbehandlung

## ✅ Zusammenfassung

Der `materialize`-Operator wandelt Benachrichtigungen in Notification-Objekte um.

- ✅ Fehler können als Daten behandelt werden
- ✅ Nützlich für Debugging und Logging
- ✅ Meta-Informationen von Benachrichtigungen können aufgezeichnet werden
- ✅ Mit `dematerialize` kann zurückgewandelt werden
- ⚠️ Fehler unterbrechen Stream nicht mehr
- ⚠️ Auf Leistungs-Overhead achten
