---
description: "Der filter-Operator sortiert Werte in einem Stream auf der Grundlage einer angegebenen bedingten Funktion und lässt nur Werte durch, die die Bedingung erfüllen. Er kann als Type-Guard-Funktion (Typ-Prädikat) in TypeScript verwendet werden. Außerdem werden der Unterschied zwischen ihm und buffer sowie die Vorbehalte bei der Umwandlung einer Prädikatfunktion in eine reine Funktion erläutert. In diesem Abschnitt wird auch der Unterschied zwischen Puffern und reinen Funktionen erklärt."
---

# filter - nur Werte durchlassen, die die Bedingungen erfüllen

Der "filter"-Operator sortiert Werte in einem Datenstrom auf der Grundlage einer angegebenen bedingten Funktion und lässt nur Werte durch, die die Bedingung erfüllen.

## 🔰 Grundlegende Syntax und Verwendung

```ts
import { from } from 'rxjs';
import { filter } from 'rxjs';

const numbers$ = from([1, 2, 3, 4, 5, 6, 7, 8, 9, 10]);

numbers$.pipe(
  filter(n => n % 2 === 0)
).subscribe(console.log);
// Ausgaben: 2, 4, 6, 8, 10
```

- Nur Werte, die der Bedingung entsprechen, werden durchgelassen.
- Funktioniert ähnlich wie `Array.prototype.filter()` auf Arrays, ist aber sequentiell auf Observable.

[🌐 Offizielle RxJS Dokumentation - `filter`](https://rxjs.dev/api/operators/filter)

## 💡 Typisches Nutzungsmuster.

- Validierung von Formulareingabewerten
- Nur Daten eines bestimmten Typs oder einer bestimmten Struktur zulassen
- Filterung von Sensorereignissen und Streamdaten

## 🧠 Praktische Code-Beispiele (mit UI)

Nur in Echtzeit auflisten, wenn die eingegebene Zahl gerade ist.

```ts
import { fromEvent } from 'rxjs';
import { map, filter } from 'rxjs';

const title = document.createElement('h3');
title.innerHTML = 'filter Praktische Beispiele für:';
document.body.appendChild(title);

// Erstellen von Eingabefeldern
const input = document.createElement('input');
input.type = 'number';
input.placeholder = 'Numerische Werte eingeben';
input.style.marginBottom = '10px';
document.body.appendChild(input);

// Ausgabebereich erstellen
const output = document.createElement('div');
document.body.appendChild(output);

// Ereignisstrom eingeben
fromEvent(input, 'input')
  .pipe(
    map((e) => parseInt((e.target as HTMLInputElement).value, 10)),
    filter((n) => !isNaN(n) && n % 2 === 0)
  )
  .subscribe((evenNumber) => {
    const item = document.createElement('div');
    item.textContent = `Erkennung gerader Zahlen: ${evenNumber}`;
    output.prepend(item);
  });

```

- Wird nur in der Ausgabe angezeigt, wenn die Zahl gerade ist.
- Ungerade oder ungültige Eingaben werden ignoriert.

> [!WARNING] 本番コードでの注意

> Das obige Beispiel lässt der Einfachheit halber das Abbestellen von `fromEvent` weg. In echtem Code sollten Sie `takeUntil(destroy$)`, `take(N)` oder `Subscription.unsubscribe()` verwenden, um den Lebenszyklus explizit zu verwalten. Weitere Informationen: [Schwierigkeiten überwinden: Lebenszyklus-Management](/de/guide/overcoming-difficulties/lifecycle-management.md)

## 🔍 Unterschiede zu buffer

| Bediener | Betrieb | Ausgabe. |
|---|---|---|
| filter". | Verwerfen von Werten, die die Bedingung nicht **erfüllen**. | Einzelne Werte `T`. |
| buffer". | Speichern** von Werten in einem Array**. | Array `T[]`. |

```ts
import { interval } from 'rxjs';
import { filter, buffer, take } from 'rxjs';

const source$ = interval(1000).pipe(take(5)); // 0, 1, 2, 3, 4

// filter - Nur Werte, die die Bedingungen erfüllen, werden durchgelassen
source$.pipe(
  filter(x => x % 2 === 0)
).subscribe(x => {
  console.log('filter:', x);
  // Ausgaben: filter: 0
  // Ausgaben: filter: 2
  // Ausgaben: filter: 4
});

// buffer - Speichert Werte als Array
source$.pipe(
  buffer(interval(2500))
).subscribe(arr => {
  console.log('buffer:', arr);
  // Ausgaben: buffer: [0, 1]
  // Ausgaben: buffer: [2, 3, 4]
});
```

## ⚠️ Anmerkungen.

### 1. Prädikatsfunktionen sollten reine Funktionen sein

Prädikatsfunktionen mit Seiteneffekten können zu unerwartetem Verhalten führen, wenn der Stream erneut abonniert wird.

```ts
// ❌ Schlechtes Beispiel: Nebeneffekte Ja
let counter = 0;
source$.pipe(
  filter(x => {
    counter++; // Nebeneffekt
    return x > 10;
  })
).subscribe();

// ✅ Gutes Beispiel: Reine Funktion
source$.pipe(
  filter(x => x > 10)
).subscribe();
```

### 2. als Typschutzfunktion verwenden

Man kann sie so schreiben, dass sie ein TypeScript-Typ-Prädikat zurückgibt (`x ist T`), um den Typ nach der Übergabe von `filter` einzugrenzen.

```ts
import { Observable, of, filter } from 'rxjs';

interface User {
  id: number;
  name: string;
  email?: string;
}

const users$: Observable<User> = of(
  { id: 1, name: 'Alice', email: 'alice@example.com' },
  { id: 2, name: 'Bob' }
);

// Wird als Typschutzfunktion verwendet
users$.pipe(
  filter((user): user is User & { email: string } => user.email !== undefined)
).subscribe(user => {
  console.log(user.email.toLowerCase()); // email ist keine Typschutzfunktion string Wird als Typ abgeleitet
});
```

> [!TIP] 型ガードの効果

> Durch die Rückgabe des Typprädikats `user is User & { email: string }` macht `user` nach `filter` `email` zu einer erforderlichen Eigenschaft. Aufrufe wie `user.email.toLowerCase()` können ohne Typfehler geschrieben werden.

## 📚 Verwandte Operatoren.

- [take](/de/guide/operators/filtering/take) - es werden nur die ersten N Werte genommen.
- [first](/de/guide/operators/filtering/first) - holt nur den ersten Wert (kann auch bedingt sein)
- [distinct](/de/guide/operators/filtering/distinct) - schließt doppelte Werte aus
- [distinctUntilChanged](/de/guide/operators/filtering/distinctUntilChanged) - schließt den gleichen wie den letzten Wert aus

## Zusammenfassung.

Der `filter` Operator ist das grundlegendste Filterwerkzeug in RxJS.

- ✅ Nur Werte, die den Bedingungen entsprechen, werden durchgelassen.
- ✅ Kann auf dieselbe Weise wie `.filter()` für Arrays verwendet werden.
- ✅ Kann auch als TypeScript type guard verwendet werden.
- ⚠️ Prädikatsfunktionen sollten reine Funktionen sein
- ⚠️ Ähnlicher Name, aber andere Verwendung als `buffer` (einzelne Werte vs. Arrays)
