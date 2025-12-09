---
description: Der tap-Operator ist ein Utility-Operator, der Nebenwirkungen ausführen kann, ohne die Werte im Stream zu beeinflussen. Er eignet sich optimal für Anwendungsfälle wie Debug-Logging, Steuerung von Ladezuständen, Analytics-Tracking und Fehlerüberwachung, bei denen externe Verarbeitung während der Beobachtung des Streams erforderlich ist. Mit TypeScript können Sie Nebenwirkungen in deklarativem Code verwalten und dabei die Typsicherheit wahren.
---

# tap - Ausführung von Seiteneffekten

Der `tap`-Operator wird verwendet, um "Seiteneffekte (Nebenwirkungen) auszuführen, ohne den Stream zu verändern".
Er eignet sich optimal für Logging, Debugging oder andere Operationen, die die Werte nicht beeinflussen.

## 🔰 Grundlegende Syntax und Funktionsweise

Er wird in Situationen eingesetzt, in denen Sie nur Nebenwirkungen hinzufügen möchten, ohne den Wertefluss zu ändern.

```ts
import { of, tap } from 'rxjs';

of(42).pipe(
  tap(value => console.log('tap:', value))
).subscribe();
// Ausgabe:
// tap: 42
```

In diesem Beispiel wird ein Log ausgegeben, wenn der von `of(42)` emittierte Wert durch `tap` fließt.
Da tap den Wert "unverändert durchlässt", beeinflusst er den Inhalt des Streams nicht.

[🌐 RxJS Offizielle Dokumentation - tap](https://rxjs.dev/api/index/function/tap)

## 💡 Typische Anwendungsfälle

`tap` wird häufig für folgende Zwecke verwendet:

- Debug-Logging
- Umschalten von Ladezuständen
- Anzeige von Toast-Benachrichtigungen
- Trigger für UI-Updates

```ts
import { of, tap, map } from 'rxjs';

of(Math.random()).pipe(
  tap(val => console.log('Erhaltener Wert:', val)),
  map(n => n > 0.5 ? 'High' : 'Low'),
  tap(label => console.log('Label:', label))
).subscribe();
// Ausgabe:
// Erhaltener Wert: 0.09909888881113504
// Label: Low
```


## 🧪 Praktisches Codebeispiel (mit UI)

Hier ist ein Beispiel, das tap verwendet, um Logs zum DOM hinzuzufügen.

```ts
import { of } from 'rxjs';
import { tap, map } from 'rxjs';

// Element für Log-Ausgabe
const logOutput = document.createElement('div');
document.body.appendChild(logOutput);

// Wertesequenz
of(1, 2, 3, 4, 5)
  .pipe(
    tap((val) => {
      console.log(`Ursprünglicher Wert: ${val}`);

      // Log zum UI hinzufügen
      const logEntry = document.createElement('div');
      logEntry.textContent = `tap: Wert ${val} ist durchgelaufen`;
      logEntry.style.color = '#666';
      logOutput.appendChild(logEntry);
    }),
    map((val) => val * 10),
    tap((val) => {
      console.log(`Transformierter Wert: ${val}`);

      // Log zum UI hinzufügen
      const logEntry = document.createElement('div');
      logEntry.textContent = `tap: Transformierter Wert ${val}`;
      logEntry.style.color = '#090';
      logOutput.appendChild(logEntry);
    })
  )
  .subscribe((val) => {
    // Endgültiges Ergebnis im UI anzeigen
    const resultItem = document.createElement('div');
    resultItem.textContent = `Ergebnis: ${val}`;
    resultItem.style.fontWeight = 'bold';
    logOutput.appendChild(resultItem);
  });

```


## ✅ Zusammenfassung

- `tap` ist ein Operator, der auf das **Einfügen von Nebenwirkungen** spezialisiert ist
- Er ermöglicht **Log-Ausgabe und UI-Updates**, ohne den Wertefluss zu ändern
- In Kombination mit `finalize` und `catchError` ist eine praktischere Steuerung möglich
