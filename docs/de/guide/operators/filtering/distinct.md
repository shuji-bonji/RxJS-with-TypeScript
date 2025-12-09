---
description: Der distinct-Operator entfernt alle doppelten Werte und gibt nur eindeutige Werte aus, die noch nie ausgegeben wurden. Da er intern ein Set verwendet, um bereits ausgegebene Werte zu speichern, ist bei unendlichen Streams Vorsicht geboten.
---

# distinct - Alle doppelten Werte entfernen

Der `distinct`-Operator überwacht alle vom Observable ausgegebenen Werte und gibt **nur Werte aus, die noch nie zuvor ausgegeben wurden**. Intern wird ein Set verwendet, um bereits ausgegebene Werte zu speichern.


## 🔰 Grundlegende Syntax und Verwendung

```ts
import { of } from 'rxjs';
import { distinct } from 'rxjs';

const numbers$ = of(1, 2, 1, 3, 2, 4, 1, 5);

numbers$.pipe(
  distinct()
).subscribe(console.log);
// Ausgabe: 1, 2, 3, 4, 5
```

- Entfernt Duplikate über den gesamten Stream
- Einmal ausgegebene Werte werden ignoriert, egal wie oft sie danach erscheinen
- `distinctUntilChanged` entfernt nur **aufeinanderfolgende** Duplikate, aber `distinct` entfernt **alle** Duplikate

[🌐 RxJS Offizielle Dokumentation - `distinct`](https://rxjs.dev/api/operators/distinct)


## 🆚 Unterschied zu distinctUntilChanged

```ts
import { of } from 'rxjs';
import { distinct, distinctUntilChanged } from 'rxjs';

const values$ = of(1, 2, 1, 2, 3, 1, 2, 3);

// distinctUntilChanged: Nur aufeinanderfolgende Duplikate entfernen
values$.pipe(
  distinctUntilChanged()
).subscribe(console.log);
// Ausgabe: 1, 2, 1, 2, 3, 1, 2, 3

// distinct: Alle Duplikate entfernen
values$.pipe(
  distinct()
).subscribe(console.log);
// Ausgabe: 1, 2, 3
```

| Operator | Entfernt | Anwendungsfall |
|---|---|---|
| `distinctUntilChanged` | Nur aufeinanderfolgende Duplikate | Eingabefelder, Sensordaten |
| `distinct` | Alle Duplikate | Liste eindeutiger Werte, ID-Listen |


## 🎯 Anpassung des Vergleichs mit keySelector

Bei Objekten können Sie mit der `keySelector`-Funktion die Duplikatserkennung nach einem bestimmten Property durchführen.

```ts
import { of } from 'rxjs';
import { distinct } from 'rxjs';

interface User {
  id: number;
  name: string;
}

const users$ = of(
  { id: 1, name: 'Alice' } as User,
  { id: 2, name: 'Bob' } as User,
  { id: 1, name: 'Alice (aktualisiert)' } as User, // Gleiche ID
  { id: 3, name: 'Charlie' } as User
);

users$.pipe(
  distinct(user => user.id) // Duplikatserkennung nach ID
).subscribe(console.log);
// Ausgabe:
// { id: 1, name: 'Alice' }
// { id: 2, name: 'Bob' }
// { id: 3, name: 'Charlie' }
```


## 💡 Typische Anwendungsmuster

1. **Eindeutige ID-Liste abrufen**
   ```ts
   import { from } from 'rxjs';
   import { distinct, map } from 'rxjs';

   interface Order {
     orderId: string;
     userId: number;
     amount: number;
   }

   const orders$ = from([
     { orderId: 'A1', userId: 1, amount: 100 },
     { orderId: 'A2', userId: 2, amount: 200 },
     { orderId: 'A3', userId: 1, amount: 150 },
     { orderId: 'A4', userId: 3, amount: 300 }
   ] as Order[]);

   // Nur eindeutige Benutzer-IDs abrufen
   orders$.pipe(
     map(order => order.userId),
     distinct()
   ).subscribe(userId => {
     console.log(`Benutzer-ID: ${userId}`);
   });
   // Ausgabe: 1, 2, 3
   ```

2. **Eindeutige Ereignistypen aus Ereignisprotokoll extrahieren**
   ```ts
   import { fromEvent, merge } from 'rxjs';
   import { map, distinct, take } from 'rxjs';

   // UI-Elemente dynamisch erstellen
   const container = document.createElement('div');
   document.body.appendChild(container);

   const button1 = document.createElement('button');
   button1.textContent = 'Button 1';
   container.appendChild(button1);

   const button2 = document.createElement('button');
   button2.textContent = 'Button 2';
   container.appendChild(button2);

   const input = document.createElement('input');
   input.placeholder = 'Bitte eingeben';
   container.appendChild(input);

   const log = document.createElement('div');
   log.style.marginTop = '10px';
   container.appendChild(log);

   // Mehrere Ereignisstreams mergen und eindeutige Ereignistypen extrahieren
   const events$ = merge(
     fromEvent(button1, 'click').pipe(map(() => 'button1-click')),
     fromEvent(button2, 'click').pipe(map(() => 'button2-click')),
     fromEvent(input, 'input').pipe(map(() => 'input-change'))
   );

   events$.pipe(
     distinct(),
     take(3) // Beendet wenn 3 Ereignistypen vollständig sind
   ).subscribe({
     next: (eventType) => {
       log.textContent += `Eindeutiges Ereignis: ${eventType}\n`;
       console.log(`Eindeutiges Ereignis: ${eventType}`);
     },
     complete: () => {
       log.textContent += 'Alle Ereignistypen erkannt';
     }
   });
   ```


## 🧠 Praktisches Codebeispiel (Tag-Eingabe)

Ein UI-Beispiel, das automatisch Duplikate aus vom Benutzer eingegebenen Tags entfernt.

```ts
import { fromEvent, Subject } from 'rxjs';
import { map, distinct, scan } from 'rxjs';

// UI-Elemente erstellen
const container = document.createElement('div');
document.body.appendChild(container);

const tagInput = document.createElement('input');
tagInput.type = 'text';
tagInput.placeholder = 'Tag eingeben und Enter drücken';
container.appendChild(tagInput);

const tagList = document.createElement('ul');
tagList.style.marginTop = '10px';
container.appendChild(tagList);

// Tag-Hinzufügen-Stream
const tagSubject$ = new Subject<string>();

tagSubject$.pipe(
  map(tag => tag.trim().toLowerCase()),
  distinct() // Duplikate-Tags entfernen
).subscribe(tag => {
  const li = document.createElement('li');
  li.textContent = tag;
  tagList.appendChild(li);
});

// Tag bei Enter-Taste hinzufügen
fromEvent<KeyboardEvent>(tagInput, 'keydown').subscribe(event => {
  if (event.key === 'Enter') {
    const value = tagInput.value.trim();
    if (value) {
      tagSubject$.next(value);
      tagInput.value = '';
    }
  }
});
```

Dieser Code stellt sicher, dass auch bei mehrfacher Eingabe desselben Tags nur einmal zur Liste hinzugefügt wird.


## ⚠️ Hinweise zur Speichernutzung

> [!WARNING]
> Der `distinct`-Operator verwendet intern ein **Set**, um alle bereits ausgegebenen Werte zu speichern. Bei unendlichen Streams kann dies zu Speicherlecks führen.

### Problem: Speicherleck bei unendlichen Streams

```ts
import { interval } from 'rxjs';
import { distinct, map } from 'rxjs';

// ❌ Schlechtes Beispiel: distinct in unendlichem Stream verwenden
interval(100).pipe(
  map(n => n % 10), // Zyklus von 0-9
  distinct() // Gibt nur die ersten 10 aus, speichert danach weiter im Speicher
).subscribe(console.log);
// Ausgabe: 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
// Danach wird nichts mehr ausgegeben, aber das Set wird weiter gehalten
```

### Lösung: Set mit flushes-Parameter löschen

```ts
import { interval, timer } from 'rxjs';
import { distinct, map } from 'rxjs';

// ✅ Gutes Beispiel: Set regelmäßig löschen
interval(100).pipe(
  map(n => n % 5),
  distinct(
    value => value,
    timer(1000) // Set alle 1 Sekunde löschen
  )
).subscribe(console.log);
// Alle 1 Sekunde werden 0, 1, 2, 3, 4 erneut ausgegeben
```

### Best Practices

1. **In endlichen Streams verwenden**: HTTP-Antworten, Array-Konvertierungen usw.
2. **flushes verwenden**: Bei unendlichen Streams regelmäßig löschen
3. **distinctUntilChanged in Betracht ziehen**: Nur aufeinanderfolgende Duplikate entfernen


## 📋 Typsichere Verwendung

Beispiel für typsichere Implementierung mit TypeScript-Generics.

```ts
import { Observable } from 'rxjs';
import { distinct, map } from 'rxjs';

interface Product {
  id: number;
  name: string;
  categoryId: number;
}

function getUniqueCategories(
  products$: Observable<Product>
): Observable<number> {
  return products$.pipe(
    distinct(product => product.categoryId)
  ).pipe(
    map(product => product.categoryId)
  );
}

// Verwendungsbeispiel
import { of } from 'rxjs';

const products$ = of(
  { id: 1, name: 'Laptop', categoryId: 10 } as Product,
  { id: 2, name: 'Maus', categoryId: 10 } as Product,
  { id: 3, name: 'Buch', categoryId: 20 } as Product
);

getUniqueCategories(products$).subscribe(categoryId => {
  console.log(`Kategorie-ID: ${categoryId}`);
});
// Ausgabe: 10, 20
```


## 🎓 Zusammenfassung

### Wann distinct verwenden
- ✅ Wenn eine Liste eindeutiger Werte benötigt wird
- ✅ Wenn Duplikate in endlichen Streams entfernt werden sollen
- ✅ Erstellung von ID-Listen oder Kategorielisten

### Wann distinctUntilChanged verwenden
- ✅ Wenn nur aufeinanderfolgende Duplikate entfernt werden sollen
- ✅ Änderungserkennung in Eingabefeldern
- ✅ Speichersparen in unendlichen Streams

### Hinweise
- ⚠️ Bei unendlichen Streams `flushes`-Parameter verwenden, um Speicherlecks zu vermeiden
- ⚠️ Bei großen Mengen eindeutiger Werte auf Speichernutzung achten
- ⚠️ Bei leistungskritischen Anwendungen Set-Größe überwachen


## 🚀 Nächste Schritte

- **[distinctUntilChanged](./distinctUntilChanged)** - Methode zum Entfernen nur aufeinanderfolgender Duplikate lernen
- **[distinctUntilKeyChanged](./distinctUntilKeyChanged)** - Methode zum Vergleichen nach Objekt-Keys lernen
- **[filter](./filter)** - Methode zum Filtern basierend auf Bedingungen lernen
- **[Praktische Beispiele für Filteroperatoren](./practical-use-cases)** - Reale Anwendungsfälle lernen
