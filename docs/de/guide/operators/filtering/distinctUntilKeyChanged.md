---
description: Der distinctUntilKeyChanged-Operator konzentriert sich auf ein bestimmtes Property in einem Objektstream und gibt nur dann aus, wenn sich dessen Wert vom vorherigen unterscheidet. Nützlich zum effizienten Überspringen aufeinanderfolgender Duplikate und zur Zustandsänderungs-Erkennung sowie Listenaktualisierungs-Optimierung.
---

# distinctUntilKeyChanged - Nur Änderungen eines bestimmten Property erkennen

Der `distinctUntilKeyChanged`-Operator konzentriert sich auf einen bestimmten Schlüssel (Property) eines Objekts und gibt nur dann aus, wenn sich dessen Wert vom vorherigen unterscheidet.
Praktisch zum effizienten Überspringen aufeinanderfolgender Duplikate.


## 🔰 Grundlegende Syntax und Verwendung

```ts
import { from } from 'rxjs';
import { distinctUntilKeyChanged } from 'rxjs';

const users = [
  { id: 1, name: 'Tanaka' },
  { id: 2, name: 'Tanaka' }, // Gleicher Name, wird übersprungen
  { id: 3, name: 'Sato' },
  { id: 4, name: 'Suzuki' },
  { id: 5, name: 'Suzuki' }, // Gleicher Name, wird übersprungen
  { id: 6, name: 'Tanaka' }
];

from(users).pipe(
  distinctUntilKeyChanged('name')
).subscribe(console.log);

// Ausgabe:
// { id: 1, name: 'Tanaka' }
// { id: 3, name: 'Sato' }
// { id: 4, name: 'Suzuki' }
// { id: 6, name: 'Tanaka' }
```

- Gibt nur aus, wenn sich der Wert des angegebenen Property `name` ändert.
- Andere Properties (z.B. `id`) werden nicht verglichen.

[🌐 RxJS Offizielle Dokumentation - `distinctUntilKeyChanged`](https://rxjs.dev/api/operators/distinctUntilKeyChanged)


## 💡 Typische Anwendungsmuster

- Bei Listenanzeige nur aktualisieren, wenn sich ein bestimmtes Property ändert
- In Ereignisstreams nur Änderungen bestimmter Attribute erkennen
- Duplikatsentfernung auf Schlüsselbasis steuern


## 🧠 Praktisches Codebeispiel (mit UI)

Namen in Textfeld eingeben und Enter drücken zum Registrieren.
**Wenn aufeinanderfolgend derselbe Name eingegeben wird, wird er ignoriert** und nur bei unterschiedlichen Namen zur Liste hinzugefügt.

```ts
import { fromEvent } from 'rxjs';
import { map, filter, scan, distinctUntilKeyChanged } from 'rxjs';

// Ausgabebereich erstellen
const output = document.createElement('div');
document.body.appendChild(output);

const title = document.createElement('h3');
title.textContent = 'Praktisches Beispiel für distinctUntilKeyChanged';
output.appendChild(title);

// Eingabeformular
const input = document.createElement('input');
input.placeholder = 'Name eingeben und Enter drücken';
document.body.appendChild(input);

// Eingabeereignisstream
fromEvent<KeyboardEvent>(input, 'keydown').pipe(
  filter((e) => e.key === 'Enter'),
  map(() => input.value.trim()),
  filter((name) => name.length > 0),
  scan((_, name, index) => ({ id: index + 1, name }), { id: 0, name: '' }),
  distinctUntilKeyChanged('name')
).subscribe((user) => {
  const item = document.createElement('div');
  item.textContent = `Benutzereingabe: ID=${user.id}, Name=${user.name}`;
  output.appendChild(item);
});
```

- Wenn aufeinanderfolgend derselbe Name eingegeben wird, wird er übersprungen.
- Wird nur angezeigt, wenn ein neuer Name eingegeben wird.
