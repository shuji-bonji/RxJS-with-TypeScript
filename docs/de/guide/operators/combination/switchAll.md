---
description: switchAll ist ein Operator, der ein Higher-order Observable (Observable of Observables) empfängt und bei jeder Ausgabe eines neuen inneren Observables zu diesem wechselt und das alte abbricht.
---

# switchAll - Zu neuem inneren Observable wechseln

Der `switchAll`-Operator empfängt ein **Higher-order Observable** (Observable of Observables),
**wechselt bei jeder Ausgabe eines neuen inneren Observables zu diesem** und bricht das alte innere Observable ab.

## 🔰 Grundlegende Syntax und Verwendung

```ts
import { fromEvent, interval } from 'rxjs';
import { map, switchAll, take } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// Bei jedem Klick einen neuen Zähler starten (Higher-order Observable)
const higherOrder$ = clicks$.pipe(
  map(() => interval(1000).pipe(take(3)))
);

// Zu neuem Zähler wechseln (alten Zähler abbrechen)
higherOrder$
  .pipe(switchAll())
  .subscribe(x => console.log(x));

// Ausgabe (bei 3 Klicks):
// 0 (1. Zähler)
// 1 (1. Zähler)
// ← Hier Klick (1. wird abgebrochen)
// 0 (2. Zähler) ← Zu neuem Zähler wechseln
// ← Hier Klick (2. wird abgebrochen)
// 0 (3. Zähler) ← Zu neuem Zähler wechseln
// 1 (3. Zähler)
// 2 (3. Zähler)
```

- **Sofortiger Wechsel**, wenn vom Higher-order Observable ein neues inneres Observable ausgegeben wird
- Das vorherige innere Observable wird **automatisch abgebrochen**
- Es wird immer nur das neueste innere Observable ausgeführt

[🌐 RxJS Official Documentation - `switchAll`](https://rxjs.dev/api/index/function/switchAll)

## 💡 Typische Anwendungsmuster

- **Suchfunktion (alte Suche bei jeder Eingabe abbrechen)**
- **Autovervollständigung**
- **Echtzeit-Datenaktualisierung (zur neuesten Datenquelle wechseln)**

## 🧠 Praktisches Codebeispiel

Ein Beispiel, das bei jeder Sucheingabe die alte Suche abbricht und nur die neueste Suche ausführt

```ts
import { fromEvent, of } from 'rxjs';
import { map, switchAll, debounceTime, delay } from 'rxjs';

const input = document.createElement('input');
input.placeholder = 'Suchbegriff eingeben';
document.body.appendChild(input);

const output = document.createElement('div');
document.body.appendChild(output);

let searchCount = 0;

// Eingabeereignis mit Debounce
const search$ = fromEvent(input, 'input').pipe(
  debounceTime(300),
  map((e) => (e.target as HTMLInputElement).value)
);

// Higher-order Observable: Simulierter Such-API-Aufruf für jeden Eingabewert
const results$ = search$.pipe(
  map(query => {
    const id = ++searchCount;
    const start = Date.now();

    // Simulierter Such-API-Aufruf (1 Sekunde Verzögerung)
    return of(`Suchergebnis: "${query}"`).pipe(
      delay(1000),
      map(msg => {
        const elapsed = ((Date.now() - start) / 1000).toFixed(1);
        return `[Suche #${id}] ${msg} (${elapsed} Sekunden)`;
      })
    );
  }),
  switchAll() // Alte Suche abbrechen, wenn neue Suche startet
);

results$.subscribe(result => {
  output.innerHTML = ''; // Vorheriges Ergebnis löschen
  const item = document.createElement('div');
  item.textContent = result;
  output.appendChild(item);
});
```

- Wenn der Benutzer die Eingabe ändert, wird **die alte Suche automatisch abgebrochen**
- Es wird immer nur das neueste Suchergebnis angezeigt

## 🔄 Verwandte Operatoren

| Operator | Beschreibung |
|---|---|
| `switchMap` | Kurzform von `map` + `switchAll` (am häufigsten verwendet) |
| [mergeAll](./mergeAll) | Alle inneren Observables parallel abonnieren |
| [concatAll](./concatAll) | Innere Observables nacheinander abonnieren (auf Abschluss des vorherigen warten) |
| [exhaustAll](./exhaustAll) | Neue innere Observables ignorieren, wenn bereits ausgeführt wird |

## ⚠️ Wichtige Hinweise

### Verhinderung von Speicherlecks

`switchAll` **bricht alte innere Observables automatisch ab**, was hilft, Speicherlecks zu verhindern.
Optimal für Suche, Autovervollständigung usw., wo häufig neue Anfragen auftreten.

### Nicht abgeschlossene innere Observables

Auch wenn innere Observables nicht abgeschlossen werden, wird automatisch gewechselt, wenn ein neues inneres Observable ausgegeben wird.

```ts
// interval wird nicht abgeschlossen, aber beim nächsten Klick automatisch abgebrochen
clicks$.pipe(
  map(() => interval(1000)), // Wird nicht abgeschlossen
  switchAll()
).subscribe();
```

### Optimal, wenn nur der neueste Wert wichtig ist

Verwenden Sie `switchAll`, wenn Ergebnisse alter Verarbeitung nicht benötigt werden und **nur das neueste Ergebnis wichtig ist**.
Wenn alle Ergebnisse benötigt werden, verwenden Sie [mergeAll](./mergeAll).
