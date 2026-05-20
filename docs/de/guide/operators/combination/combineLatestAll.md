---
description: "combineLatestAll ist ein Operator, der eine Observable höherer Ordnung (Observable of Observable) nimmt und den jeweils neuesten Wert kombiniert, sobald alle internen Observable mindestens einmal ausgelöst wurden."
---

# combineLatestAll - kombiniert die letzten Werte des internen Observable

Der Operator "combineLatestAll" nimmt ein **Observable höherer Ordnung** (Observable of Observables),
**Wenn alle internen Observable mindestens einmal ausgelöst haben**, werden die **letzten Werte** jedes Observable kombiniert und als **Array** ausgegeben.

## 🔰 Grundlegende Syntax und Verwendung

```ts
import { interval, of } from 'rxjs';
import { combineLatestAll, take } from 'rxjs';

// 3Zwei interneObservablemitHigher-order Observable
const higherOrder$ = of(
  interval(1000).pipe(take(3)), // 0, 1, 2
  interval(500).pipe(take(4)),  // 0, 1, 2, 3
  interval(2000).pipe(take(2))  // 0, 1
);

// Alle internenObservablehaben mindestens1einmal gefeuert, kombinieren Sie die letzten Werte
higherOrder$
  .pipe(combineLatestAll())
  .subscribe(values => console.log(values));

// Ausgabe:
// [1, 3, 0] ← Wenn alle mindestens einmal gezündet haben1Wenn alle mindestens einmal gezündet haben (nach2Sekunden später)
// [2, 3, 0] ← 1Wenn der zweiteObservableabgefeuert wird (nach 2 Sekunden)2feuert (nach3Sekunden später)
// [2, 3, 1] ← 3Wenn der zweiteObservableabgefeuert wird (nach 2 Sekunden)1feuert (nach4Sekunden später)
```

- Interne Observables sammeln, wenn Observable höherer Ordnung **vollständig** ist.
- **Beginnen Sie mit dem Kombinieren, wenn alle internen Observable mindestens einmal ausgelöst haben**.
- Immer wenn eine interne Observable einen Wert ausgibt, **kombinieren** Sie alle letzten Werte und **Ausgaben**.

[🌐 Offizielle RxJS Dokumentation - `combineLatestAll`](https://rxjs.dev/api/index/function/combineLatestAll)

## 💡 Typisches Nutzungsmuster.

- **Die neuesten Ergebnisse mehrerer API-Aufrufe zusammenfassen**
- **Synchronisieren der letzten Werte mehrerer Formulareingaben**
- **Mehrere Echtzeit-Datenquellen einbinden**

## 🧠 Praktische Code-Beispiele

Beispiel für die Kombination der letzten Ergebnisse mehrerer API-Aufrufe

```ts
import { of, timer, Observable } from 'rxjs';
import { map, combineLatestAll, take } from 'rxjs';

const output = document.createElement('div');
document.body.appendChild(output);

// 3Zwei simulierteAPIAufruf erstellen (Higher-order Observable)
const apiCalls$: Observable<Observable<string>> = of(
  // API 1: Benutzerinformationen (1Abgeschlossen in Sekunden,3einmal aktualisiert)
  timer(0, 1000).pipe(
    take(3),
    map(n => `Benutzer: User${n}`)
  ),
  // API 2: Anzahl der Benachrichtigungen (0.5Abgeschlossen in Sekunden,4einmal aktualisiert)
  timer(0, 500).pipe(
    take(4),
    map(n => `Benachrichtigungen: ${n}Anzahl der Benachrichtigungen`)
  ),
  // API 3: Status2Abgeschlossen in Sekunden,2einmal aktualisiert)
  timer(0, 2000).pipe(
    take(2),
    map(n => n === 0 ? 'Status: Offline' : 'Status: Online')
  )
);

// AlleAPIKombinierter letzter Wert der Anrufe
apiCalls$
  .pipe(combineLatestAll())
  .subscribe(values => {
    output.innerHTML = '<strong>Letzter Status:</strong><br>';
    values.forEach((value, index) => {
      const item = document.createElement('div');
      item.textContent = `${index + 1}. ${value}`;
      output.appendChild(item);
    });
  });
```

- Drei API-Aufrufe werden parallel ausgeführt.
- **Nachdem alle mindestens einmal ausgelöst wurden**, werden die kombinierten Ergebnisse angezeigt
- Jedes Mal, wenn eine API aktualisiert wird, wird die **neueste Kombination** angezeigt

## 🔄 Verwandte Creation Function

Die Funktion **combineLatestAll** wird hauptsächlich für die Verflachung von Observable höherer Ordnung verwendet,
Verwenden Sie die **Creation Function** `combineLatest` für normale Kombinationen aus Observables höherer Ordnung.

```ts
import { combineLatest, interval } from 'rxjs';

// Creation FunctionAusgabe (allgemeinere Verwendung)
const combined$ = combineLatest([
  interval(1000),
  interval(500),
  interval(2000)
]);

combined$.subscribe(console.log);
```

Siehe [Kapitel 3 Creation Function - combineLatest](/de/guide/creation-functions/combination/combineLatest).

## 🔄 Verwandte Operatoren.

| Bediener. | Beschreibung. |
|---|---|
| [mergeAll](./mergeAll) | Alle internen Observable parallel abonnieren. |
| [concatAll](./concatAll) | Interne Observable nacheinander abonnieren. |
| [switchAll](./switchAll) | Zu einem neuen internen Observable wechseln. |
| [zipAll](./zipAll) | Paarung der Werte jedes internen Observable in der entsprechenden Reihenfolge |

## ⚠️ Anmerkungen.

### Observable höherer Ordnung muss ausgefüllt werden.

combineLatestAll" wartet auf die Sammlung interner Observables, **bis** die Observable höherer Ordnung (äußere Observable) **vollständig** ist.

#### ❌ Observable höherer Ordnung wird nicht abgeschlossen, also wird nichts ausgegeben

```ts
interval(1000).pipe(
  map(() => of(1, 2, 3)),
  combineLatestAll()
).subscribe(console.log); // Es wird nichts ausgegeben
```

#### ✅ Take to complete

```ts
interval(1000).pipe(
  take(3), // 3Abgeschlossen in einer Sitzung
  map(() => of(1, 2, 3)),
  combineLatestAll()
).subscribe(console.log);
```

### Alle internen Observable müssen mindestens einmal ausgelöst werden

Es wird kein Wert ausgegeben, bevor nicht alle internen Observable **mindestens einmal** ausgelöst haben.

```ts
// 1Wenn einer der internenObservableEs wird nichts ausgegeben, wenn es
of(
  of(1, 2, 3),
  NEVER // Niemals feuert für immer.
).pipe(
  combineLatestAll()
).subscribe(console.log); // Es wird nichts ausgegeben
```

### Speicherverbrauch.

Beachten Sie den Speicherverbrauch, wenn es viele interne Observable gibt, da die **letzten Werte aller internen Observable im Speicher gehalten werden**.
