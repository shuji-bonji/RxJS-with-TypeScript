---
description: combineLatestAll ist ein Operator, der ein Higher-order Observable (Observable of Observables) empfängt und, wenn alle inneren Observables mindestens einmal ausgelöst haben, die neuesten Werte jedes einzelnen kombiniert und als Array ausgibt.
---

# combineLatestAll - Neueste Werte aller inneren Observables kombinieren

Der `combineLatestAll`-Operator empfängt ein **Higher-order Observable** (Observable of Observables),
und wenn **alle inneren Observables mindestens einmal ausgelöst haben**, gibt er die **neuesten Werte jedes einzelnen** kombiniert als Array aus.

## 🔰 Grundlegende Syntax und Verwendung

```ts
import { interval, of } from 'rxjs';
import { combineLatestAll, take } from 'rxjs';

// Higher-order Observable mit 3 inneren Observables
const higherOrder$ = of(
  interval(1000).pipe(take(3)), // 0, 1, 2
  interval(500).pipe(take(4)),  // 0, 1, 2, 3
  interval(2000).pipe(take(2))  // 0, 1
);

// Wenn alle inneren Observables mindestens einmal ausgelöst haben, neueste Werte kombinieren
higherOrder$
  .pipe(combineLatestAll())
  .subscribe(values => console.log(values));

// Ausgabe:
// [1, 3, 0] ← Wenn alle mindestens einmal ausgelöst haben (nach 2 Sekunden)
// [2, 3, 0] ← Wenn 1. Observable 2 ausgibt (nach 3 Sekunden)
// [2, 3, 1] ← Wenn 3. Observable 1 ausgibt (nach 4 Sekunden)
```

- Sammelt innere Observables, wenn das Higher-order Observable **abgeschlossen** ist
- Wenn **alle inneren Observables mindestens einmal ausgelöst haben**, beginnt die Kombination
- Jedes Mal, wenn eines der inneren Observables einen Wert ausgibt, werden **alle neuesten Werte kombiniert** und ausgegeben

[🌐 RxJS Official Documentation - `combineLatestAll`](https://rxjs.dev/api/index/function/combineLatestAll)

## 💡 Typische Anwendungsmuster

- **Neueste Ergebnisse mehrerer API-Aufrufe kombinieren**
- **Neueste Werte mehrerer Formulareingaben synchronisieren**
- **Mehrere Echtzeit-Datenquellen integrieren**

## 🧠 Praktisches Codebeispiel

Ein Beispiel, das neueste Ergebnisse mehrerer API-Aufrufe kombiniert und anzeigt

```ts
import { of, timer, Observable } from 'rxjs';
import { map, combineLatestAll, take } from 'rxjs';

const output = document.createElement('div');
document.body.appendChild(output);

// 3 simulierte API-Aufrufe erstellen (Higher-order Observable)
const apiCalls$: Observable<Observable<string>> = of(
  // API 1: Benutzerinformationen (nach 1 Sekunde abgeschlossen, 3 Aktualisierungen)
  timer(0, 1000).pipe(
    take(3),
    map(n => `Benutzer: User${n}`)
  ),
  // API 2: Benachrichtigungsanzahl (nach 0,5 Sekunden abgeschlossen, 4 Aktualisierungen)
  timer(0, 500).pipe(
    take(4),
    map(n => `Benachrichtigungen: ${n} Stück`)
  ),
  // API 3: Status (nach 2 Sekunden abgeschlossen, 2 Aktualisierungen)
  timer(0, 2000).pipe(
    take(2),
    map(n => n === 0 ? 'Status: Offline' : 'Status: Online')
  )
);

// Neueste Werte aller API-Aufrufe kombinieren und anzeigen
apiCalls$
  .pipe(combineLatestAll())
  .subscribe(values => {
    output.innerHTML = '<strong>Neuester Zustand:</strong><br>';
    values.forEach((value, index) => {
      const item = document.createElement('div');
      item.textContent = `${index + 1}. ${value}`;
      output.appendChild(item);
    });
  });
```

- 3 API-Aufrufe werden parallel ausgeführt
- Wenn **alle mindestens einmal ausgelöst haben**, wird das kombinierte Ergebnis angezeigt
- Jedes Mal, wenn eine API aktualisiert wird, wird die **neueste Kombination** angezeigt

## 🔄 Verwandte Creation Function

`combineLatestAll` wird hauptsächlich zur Vereinfachung von Higher-order Observables verwendet,
aber für die normale Kombination mehrerer Observables verwenden Sie die **Creation Function** `combineLatest`.

```ts
import { combineLatest, interval } from 'rxjs';

// Creation Function-Version (häufigere Verwendung)
const combined$ = combineLatest([
  interval(1000),
  interval(500),
  interval(2000)
]);

combined$.subscribe(console.log);
```

Siehe [Kapitel 3 Creation Functions - combineLatest](/de/guide/creation-functions/combination/combineLatest).

## 🔄 Verwandte Operatoren

| Operator | Beschreibung |
|---|---|
| [mergeAll](./mergeAll) | Alle inneren Observables parallel abonnieren |
| [concatAll](./concatAll) | Innere Observables nacheinander abonnieren |
| [switchAll](./switchAll) | Zu neuem inneren Observable wechseln |
| [zipAll](./zipAll) | Entsprechende Werte jedes inneren Observables paaren |

## ⚠️ Wichtige Hinweise

### Abschluss des Higher-order Observable erforderlich

`combineLatestAll` wartet, bis das Higher-order Observable (äußeres Observable) **abgeschlossen** ist, bevor es innere Observables sammelt.

#### ❌ Higher-order Observable wird nicht abgeschlossen, daher keine Ausgabe
```ts
interval(1000).pipe(
  map(() => of(1, 2, 3)),
  combineLatestAll()
).subscribe(console.log); // Keine Ausgabe
```

#### ✅ Mit take abschließen
```ts
interval(1000).pipe(
  take(3), // Mit 3 abschließen
  map(() => of(1, 2, 3)),
  combineLatestAll()
).subscribe(console.log);
```

### Alle inneren Observables müssen mindestens einmal auslösen

Es wird nichts ausgegeben, **bis alle inneren Observables mindestens einmal ausgelöst haben**.

```ts
// Wenn auch nur ein inneres Observable nicht auslöst, keine Ausgabe
of(
  of(1, 2, 3),
  NEVER // Löst niemals aus
).pipe(
  combineLatestAll()
).subscribe(console.log); // Keine Ausgabe
```

### Speicherverbrauch

Da die **neuesten Werte aller inneren Observables im Speicher gehalten** werden, achten Sie bei vielen inneren Observables auf den Speicherverbrauch.
