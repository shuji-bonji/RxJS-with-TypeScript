---
description: dematerialize ist ein RxJS Utility-Operator, der Notification-Objekte in normale Benachrichtigungen (next, error, complete) zurückwandelt und die inverse Umwandlung von materialize durchführt. Er eignet sich optimal für Szenarien, in denen nach Verarbeitung von Benachrichtigungen als Daten eine Rückwandlung zum Originalformat gewünscht ist, wie Wiederherstellung nach Benachrichtigungsbearbeitung, Filterung oder Transformation von Fehlern oder Neuordnung und Pufferung von Benachrichtigungen.
---

# dematerialize - Notification Umwandeln

Der `dematerialize`-Operator **wandelt Notification-Objekte in normale Benachrichtigungen (next, error, complete)** um. Er führt die inverse Umwandlung von `materialize` durch und stellt datifizierte Benachrichtigungen in ihre ursprüngliche Form wieder her.

## 🔰 Grundlegende Syntax und Funktionsweise

Wandelt einen Stream von Notification-Objekten zurück in einen normalen Stream.

```ts
import { of } from 'rxjs';
import { materialize, dematerialize } from 'rxjs';

of(1, 2, 3)
  .pipe(
    materialize(),     // In Notification-Objekte umwandeln
    dematerialize()    // Zurückwandeln
  )
  .subscribe({
    next: v => console.log('Wert:', v),
    complete: () => console.log('Abgeschlossen')
  });
// Ausgabe:
// Wert: 1
// Wert: 2
// Wert: 3
// Abgeschlossen
```

[🌐 RxJS Offizielle Dokumentation - dematerialize](https://rxjs.dev/api/index/function/dematerialize)

## 💡 Typische Anwendungsfälle

- **Wiederherstellung nach Benachrichtigungsbearbeitung**: Nach Verarbeitung mit materialize zurück ins Originalformat
- **Fehlerfilterung**: Ausschließen nur bestimmter Fehler
- **Neuordnung von Benachrichtigungen**: Wiederherstellung nach Sortierung von Benachrichtigungen als Daten
- **Wiederherstellung nach Debugging**: Nach Log-Ausgabe usw. Rückkehr zum normalen Betrieb

## 🧪 Praktisches Codebeispiel 1: Selektive Fehlerfilterung

Beispiel für Ausschließen nur bestimmter Fehler und normale Verarbeitung der übrigen.

```ts
import { of, throwError, concat } from 'rxjs';
import { materialize, dematerialize, filter } from 'rxjs';

// UI erstellen
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'dematerialize - Fehlerfilterung';
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

// Stream mit Fehlern
const source$ = concat(
  of(1, 2),
  throwError(() => new Error('Zu ignorierender Fehler')),
  of(3, 4),
  throwError(() => new Error('Schwerwiegender Fehler')),
  of(5)
);

source$
  .pipe(
    materialize(),
    filter(notification => {
      // Nur "zu ignorierende Fehler" filtern
      if (notification.kind === 'E') {
        const errorMessage = notification.error?.message || '';
        if (errorMessage.includes('ignorieren')) {
          addLog(`🔇 Ignoriert: ${errorMessage}`, '#fff9c4');
          return false;  // Diesen Fehler ausschließen
        }
      }
      return true;
    }),
    dematerialize()  // Zurück ins Originalformat
  )
  .subscribe({
    next: v => addLog(`✅ Wert: ${v}`, '#c8e6c9'),
    error: err => addLog(`❌ Fehler: ${err.message}`, '#ffcdd2'),
    complete: () => addLog('Abgeschlossen', '#e3f2fd')
  });
```

- "Zu ignorierende Fehler" werden ausgeschlossen und Stream wird fortgesetzt
- "Schwerwiegende Fehler" werden normal an Fehlerhandler übergeben
- Selektive Fehlerverarbeitung möglich

## 🧪 Praktisches Codebeispiel 2: Verzögerte Verarbeitung von Benachrichtigungen

Beispiel für vorübergehendes Puffern von Benachrichtigungen vor Wiederherstellung.

```ts
import { from, interval, take, delay } from 'rxjs';
import { materialize, dematerialize, bufferTime, concatMap } from 'rxjs';

// UI erstellen
const container2 = document.createElement('div');
container2.style.marginTop = '20px';
document.body.appendChild(container2);

const title2 = document.createElement('h3');
title2.textContent = 'dematerialize - Pufferung und Verzögerung';
container2.appendChild(title2);

const output2 = document.createElement('div');
output2.style.border = '1px solid #ccc';
output2.style.padding = '10px';
output2.style.maxHeight = '200px';
output2.style.overflow = 'auto';
container2.appendChild(output2);

function addLog2(message: string) {
  const now = new Date();
  const timestamp = now.toLocaleTimeString('de-DE', { hour12: false }) +
    '.' + now.getMilliseconds().toString().padStart(3, '0');

  const logItem = document.createElement('div');
  logItem.textContent = `[${timestamp}] ${message}`;
  output2.appendChild(logItem);
}

addLog2('Start - Werte jede Sekunde emittieren, alle 2 Sekunden zusammen verarbeiten');

interval(1000)
  .pipe(
    take(6),
    materialize(),
    bufferTime(2000),      // Alle 2 Sekunden puffern
    concatMap(notifications => {
      addLog2(`--- ${notifications.length} Benachrichtigungen aus Puffer verarbeiten ---`);
      return from(notifications).pipe(
        delay(500),        // Jede Benachrichtigung 0,5 Sekunden verzögern
        dematerialize()    // Zurück ins Originalformat
      );
    })
  )
  .subscribe({
    next: v => addLog2(`Wert: ${v}`),
    complete: () => addLog2('Abgeschlossen')
  });
```

- Benachrichtigungen alle 2 Sekunden puffern
- Aus Puffer entnehmen und verzögert verarbeiten
- Mit `dematerialize` als ursprünglicher Stream wiederherstellen

## 🆚 Beziehung zu materialize

```ts
import { of } from 'rxjs';
import { materialize, dematerialize, map } from 'rxjs';

of(1, 2, 3)
  .pipe(
    materialize(),           // In Notification umwandeln
    map(notification => {
      // Als Notification-Objekt verarbeiten
      console.log('kind:', notification.kind);
      return notification;
    }),
    dematerialize()          // Zurückwandeln
  )
  .subscribe(v => console.log('Wert:', v));
// Ausgabe:
// kind: N
// Wert: 1
// kind: N
// Wert: 2
// kind: N
// Wert: 3
// kind: C
```

| Verarbeitungsablauf | Erklärung |
|:---|:---|
| Ursprünglicher Stream | Normale Werte (next), Fehler (error), Abschluss (complete) |
| ↓ `materialize()` | Stream von Notification-Objekten |
| Zwischenverarbeitung | Bearbeitung/Filterung als Notification |
| ↓ `dematerialize()` | Wiederherstellung als normaler Stream |
| Endgültiger Stream | Normale Werte, Fehler, Abschluss |

## ⚠️ Wichtige Hinweise

### 1. Fehlerbenachrichtigungen werden in echte Fehler umgewandelt

```ts
import { of, throwError, concat } from 'rxjs';
import { materialize, dematerialize } from 'rxjs';

// Jedes Observable mit materialize() in Benachrichtigungsobjekt umwandeln
concat(
  of(1).pipe(materialize()),
  throwError(() => new Error('Fehler')).pipe(materialize()),
  of(2).pipe(materialize())  // Nach Fehler nicht ausgeführt
)
  .pipe(
    dematerialize()
  )
  .subscribe({
    next: v => console.log('Wert:', v),
    error: err => console.log('Fehler:', err.message)
  });
// Ausgabe:
// Wert: 1
// Fehler: Fehler
```

Bei Erreichen von Fehlerbenachrichtigung wird Stream mit Fehler unterbrochen.

### 2. Stream wird bei Abschlussbenachrichtigung abgeschlossen

```ts
import { of, EMPTY, concat } from 'rxjs';
import { materialize, dematerialize } from 'rxjs';

// Jedes Observable mit materialize() in Benachrichtigungsobjekt umwandeln
concat(
  of(1).pipe(materialize()),
  of(2).pipe(materialize()),
  EMPTY.pipe(materialize()),  // Abschlussbenachrichtigung
  of(3).pipe(materialize())   // Nach Abschluss nicht ausgeführt
)
  .pipe(
    dematerialize()
  )
  .subscribe({
    next: v => console.log('Wert:', v),
    complete: () => console.log('Abgeschlossen')
  });
// Ausgabe:
// Wert: 1
// Wert: 2
// Abgeschlossen
```

Werte nach Abschlussbenachrichtigung werden nicht emittiert.

### 3. Ungültige Notification-Objekte

`dematerialize` erwartet korrekte Notification-Objekte.

```ts
import { of } from 'rxjs';
import { dematerialize } from 'rxjs';

// ❌ Normale Werte an dematerialize übergeben führt zu Fehler
of(1, 2, 3)
  .pipe(
    dematerialize()  // Keine Notification-Objekte
  )
  .subscribe({
    next: console.log,
    error: err => console.error('Fehler:', err.message)
  });
// Fehler tritt auf
```

## Praktisches Kombinationsbeispiel

```ts
import { interval, throwError, of, concat } from 'rxjs';
import { materialize, dematerialize, take, mergeMap, map } from 'rxjs';

// Beispiel für Umwandlung von Fehlern in Warnungen
interval(500)
  .pipe(
    take(10),
    mergeMap(value => {
      // Nur bei 5 Fehler auslösen
      if (value === 5) {
        return throwError(() => new Error(`Fehler bei Wert ${value}`));
      }
      return of(value);
    }),
    materialize(),
    map(notification => {
      // Fehler in Warnungsmeldungen umwandeln
      if (notification.kind === 'E') {
        console.warn('Warnung:', notification.error?.message);
        // Anstelle von Fehler speziellen Wert emittieren (mit materialize() erzeugt)
        return { kind: 'N' as const, value: -1 };
      }
      return notification;
    }),
    dematerialize()
  )
  .subscribe({
    next: v => console.log('Wert:', v),
    error: err => console.error('Fehler:', err),  // Wird nicht aufgerufen
    complete: () => console.log('Abgeschlossen')
  });
// Ausgabe:
// Wert: 0, 1, 2, 3, 4
// Warnung: Fehler bei Wert 5
// Wert: -1  (Anstelle von Fehler)
// Wert: 6, 7, 8, 9
// Abgeschlossen
```

## 📚 Verwandte Operatoren

- **[materialize](./materialize)** - Umwandlung von Benachrichtigungen in Notification-Objekte
- **[catchError](../../error-handling/retry-catch)** - Fehlerbehandlung
- **[retry](./retry)** - Wiederholung bei Fehler

## ✅ Zusammenfassung

Der `dematerialize`-Operator wandelt Notification-Objekte zurück in normale Benachrichtigungen.

- ✅ Inverse Umwandlung von `materialize`
- ✅ Wiederherstellung ins Originalformat nach Benachrichtigungsbearbeitung
- ✅ Filterung und Transformation von Fehlern möglich
- ✅ Verwendung für Neuordnung und Pufferung von Benachrichtigungen
- ⚠️ Fehlerbenachrichtigungen wirken als echte Fehler
- ⚠️ Stream wird bei Abschlussbenachrichtigung abgeschlossen
- ⚠️ Korrekte Notification-Objekte erforderlich
