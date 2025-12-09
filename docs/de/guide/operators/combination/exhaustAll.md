---
description: exhaustAll ist ein Operator, der ein Higher-order Observable (Observable of Observables) empfängt und neue innere Observables ignoriert, wenn bereits eines ausgeführt wird.
---

# exhaustAll - Neue innere Observables ignorieren, wenn bereits ausgeführt wird

Der `exhaustAll`-Operator empfängt ein **Higher-order Observable** (Observable of Observables),
**ignoriert neue innere Observables, wenn bereits eines ausgeführt wird**.

## 🔰 Grundlegende Syntax und Verwendung

```ts
import { fromEvent, interval } from 'rxjs';
import { map, exhaustAll, take } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// Bei jedem Klick einen neuen Zähler starten (Higher-order Observable)
const higherOrder$ = clicks$.pipe(
  map(() => interval(1000).pipe(take(3)))
);

// Neue Klicks ignorieren, wenn Zähler ausgeführt wird
higherOrder$
  .pipe(exhaustAll())
  .subscribe(x => console.log(x));

// Ausgabe (bei 3 aufeinanderfolgenden Klicks):
// 0 (1. Zähler)
// 1 (1. Zähler)
// ← Hier Klick (wird ignoriert: 1. wird noch ausgeführt)
// 2 (1. Zähler) ← Abgeschlossen
// ← Hier Klick (wird akzeptiert: kein Zähler wird ausgeführt)
// 0 (2. Zähler)
// 1 (2. Zähler)
// 2 (2. Zähler)
```

- Wenn ein inneres Observable ausgeführt wird, werden **neue innere Observables ignoriert**
- Das nächste wird erst **nach Abschluss des ausgeführten Observables akzeptiert**
- Optimal zur Verhinderung doppelter Ausführung

[🌐 RxJS Official Documentation - `exhaustAll`](https://rxjs.dev/api/index/function/exhaustAll)

## 💡 Typische Anwendungsmuster

- **Doppelklick-Verhinderung (Button-Mehrfachklick-Verhinderung)**
- **Verhinderung doppelter Übermittlung von Login-Anfragen**
- **Verhinderung doppelter Ausführung von Speichervorgängen**

## 🧠 Praktisches Codebeispiel

Ein Beispiel zur Verhinderung von Doppelklicks auf Speicher-Button

```ts
import { fromEvent, of } from 'rxjs';
import { map, exhaustAll, delay } from 'rxjs';

const saveButton = document.createElement('button');
saveButton.textContent = 'Speichern';
document.body.appendChild(saveButton);

const output = document.createElement('div');
document.body.appendChild(output);

let saveCount = 0;

// Button-Klick-Ereignis
const clicks$ = fromEvent(saveButton, 'click');

// Higher-order Observable: Simulierter Speichervorgang für jeden Klick
const saves$ = clicks$.pipe(
  map(() => {
    const id = ++saveCount;
    const start = Date.now();

    // Button vorübergehend deaktivieren (visuelles Feedback)
    saveButton.disabled = true;

    // Simulierter Speichervorgang (2 Sekunden Verzögerung)
    return of(`Speichern abgeschlossen #${id}`).pipe(
      delay(2000),
      map(msg => {
        saveButton.disabled = false;
        const elapsed = ((Date.now() - start) / 1000).toFixed(1);
        return `${msg} (${elapsed} Sekunden)`;
      })
    );
  }),
  exhaustAll() // Neue Klicks während des Speicherns ignorieren
);

saves$.subscribe(result => {
  const item = document.createElement('div');
  item.textContent = result;
  output.prepend(item);
});

// Ignorierte Klicks in Log ausgeben
clicks$.subscribe(() => {
  if (saveButton.disabled) {
    console.log('Klick wurde ignoriert, da Speichervorgang läuft');
  }
});
```

- Während des Speichervorgangs werden **neue Klicks ignoriert**
- Nach Abschluss des Speichervorgangs wird der nächste Klick akzeptiert

## 🔄 Verwandte Operatoren

| Operator | Beschreibung |
|---|---|
| `exhaustMap` | Kurzform von `map` + `exhaustAll` (häufig verwendet) |
| [mergeAll](./mergeAll) | Alle inneren Observables parallel abonnieren |
| [concatAll](./concatAll) | Innere Observables nacheinander abonnieren (in Warteschlange einreihen) |
| [switchAll](./switchAll) | Zu neuem inneren Observable wechseln (altes abbrechen) |

## 🔄 Vergleich mit anderen Operatoren

| Operator | Wenn neues inneres Observable ausgegeben wird |
|---|---|
| `mergeAll` | Parallel ausführen |
| `concatAll` | Zur Warteschlange hinzufügen (auf Abschluss des vorherigen warten) |
| `switchAll` | Altes abbrechen und wechseln |
| `exhaustAll` | **Ignorieren (auf Abschluss des ausgeführten warten)** |

## ⚠️ Wichtige Hinweise

### Verlust von Ereignissen

`exhaustAll` **ignoriert Ereignisse während der Ausführung vollständig**, daher ungeeignet, wenn alle Ereignisse verarbeitet werden sollen.

```ts
// ❌ Wenn alle Klicks aufgezeichnet werden sollen, ist exhaustAll ungeeignet
// ✅ mergeAll oder concatAll verwenden
```

### UI-Feedback

Es ist wichtig, dem Benutzer visuell zu vermitteln, dass "Ereignisse ignoriert wurden".

```ts
// Button deaktivieren
saveButton.disabled = true;

// Toast-Nachricht anzeigen
showToast('Verarbeitung läuft. Bitte warten Sie.');
```

### Geeignete Anwendungsfälle

#### Fälle, in denen `exhaustAll` optimal ist
- Login-Verarbeitung (Verhinderung doppelter Übermittlung)
- Speichervorgang (Verhinderung doppelter Ausführung)
- Animation (keine neue Animation starten, wenn bereits ausgeführt wird)

#### Fälle, in denen `exhaustAll` ungeeignet ist
- Suchverarbeitung (neueste Suche soll ausgeführt werden → `switchAll`)
- Alle Ereignisse sollen verarbeitet werden (→ `mergeAll` oder `concatAll`)
