---
description: concatAll ist ein Operator, der ein Higher-order Observable (Observable of Observables) empfängt und innere Observables nacheinander abonniert, um Werte zu vereinfachen. Das nächste wird erst gestartet, nachdem das vorherige abgeschlossen ist.
---

# concatAll - Innere Observables nacheinander vereinfachen

Der `concatAll`-Operator empfängt ein **Higher-order Observable** (Observable of Observables),
**abonniert innere Observables nacheinander** und vereinfacht die Werte. Das nächste startet nicht, bevor das vorherige abgeschlossen ist.

## 🔰 Grundlegende Syntax und Verwendung

```ts
import { fromEvent, interval } from 'rxjs';
import { map, concatAll, take } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// Bei jedem Klick einen neuen Zähler starten (Higher-order Observable)
const higherOrder$ = clicks$.pipe(
  map(() => interval(1000).pipe(take(3)))
);

// Zähler nacheinander abonnieren (nächster startet erst nach Abschluss des vorherigen)
higherOrder$
  .pipe(concatAll())
  .subscribe(x => console.log(x));

// Ausgabe (bei 3 Klicks):
// 0 (1. Zähler)
// 1 (1. Zähler)
// 2 (1. Zähler) ← abgeschlossen
// 0 (2. Zähler) ← startet nach Abschluss des 1.
// 1 (2. Zähler)
// 2 (2. Zähler) ← abgeschlossen
// 0 (3. Zähler) ← startet nach Abschluss des 2.
// ...
```

- Jedes innere Observable vom Higher-order Observable wird **nacheinander abonniert**
- Das nächste startet nicht, **bevor das innere Observable abgeschlossen ist**
- Wertereihenfolge ist garantiert

[🌐 RxJS Official Documentation - `concatAll`](https://rxjs.dev/api/index/function/concatAll)

## 💡 Typische Anwendungsmuster

- **API-Aufrufe nacheinander ausführen (nächste Anfrage erst nach Abschluss der vorherigen)**
- **Animationen nacheinander abspielen**
- **Datei-Uploads nacheinander verarbeiten**

## 🧠 Praktisches Codebeispiel

Ein Beispiel, das bei jedem Button-Klick API-Aufrufe (simuliert) nacheinander ausführt

```ts
import { fromEvent, of } from 'rxjs';
import { map, concatAll, delay } from 'rxjs';

const button = document.createElement('button');
button.textContent = 'API-Aufruf';
document.body.appendChild(button);

const output = document.createElement('div');
document.body.appendChild(output);

let callCount = 0;

// Button-Klick-Ereignis
const clicks$ = fromEvent(button, 'click');

// Higher-order Observable: Simulierter API-Aufruf für jeden Klick
const results$ = clicks$.pipe(
  map(() => {
    const id = ++callCount;
    const start = Date.now();

    // Simulierter API-Aufruf (2 Sekunden Verzögerung)
    return of(`API-Aufruf #${id} abgeschlossen`).pipe(
      delay(2000),
      map(msg => {
        const elapsed = ((Date.now() - start) / 1000).toFixed(1);
        return `${msg} (${elapsed} Sekunden)`;
      })
    );
  }),
  concatAll() // Alle API-Aufrufe nacheinander ausführen
);

results$.subscribe(result => {
  const item = document.createElement('div');
  item.textContent = result;
  output.prepend(item);
});
```

- Auch bei aufeinanderfolgenden Button-Klicks werden **API-Aufrufe nacheinander ausgeführt**
- Der nächste API-Aufruf startet erst nach Abschluss des vorherigen

## 🔄 Verwandte Operatoren

| Operator | Beschreibung |
|---|---|
| `concatMap` | Kurzform von `map` + `concatAll` (häufig verwendet) |
| [mergeAll](./mergeAll) | Alle inneren Observables parallel abonnieren |
| [switchAll](./switchAll) | Zu neuem inneren Observable wechseln (altes abbrechen) |
| [exhaustAll](./exhaustAll) | Neue innere Observables ignorieren, wenn bereits ausgeführt wird |

## ⚠️ Wichtige Hinweise

### Backpressure (Stau)

Wenn die Ausgabegeschwindigkeit innerer Observables höher ist als die Abschlussgeschwindigkeit, **sammeln sich unverarbeitete Observables in der Warteschlange**.

```ts
// Klick jede Sekunde → API-Aufruf dauert 2 Sekunden
// → Könnte sich weiter in der Warteschlange ansammeln
```

In diesem Fall sollten folgende Gegenmaßnahmen in Betracht gezogen werden
- `switchAll` verwenden (nur neuestes verarbeiten)
- `exhaustAll` verwenden (während Ausführung ignorieren)
- Debouncing oder Throttling hinzufügen

### Vorsicht bei unendlichen Observables

Wenn das vorherige Observable **nicht abgeschlossen wird, startet das nächste niemals**.

#### ❌ interval wird nicht abgeschlossen, daher startet der 2. Zähler nicht
```ts
clicks$.pipe(
  map(() => interval(1000)), // Wird nicht abgeschlossen
  concatAll()
).subscribe();
```
#### ✅ Mit take abschließen
```ts
clicks$.pipe(
  map(() => interval(1000).pipe(take(3))), // Mit 3 abschließen
  concatAll()
).subscribe();
```
