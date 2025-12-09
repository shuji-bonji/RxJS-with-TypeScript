---
description: "Eine Einführung in die Einrichtung einer Lernentwicklungsvorlage bestehend aus Vite, TypeScript und RxJS. Hot-Reload-Unterstützung ermöglicht sofortige Code-Experimente und DOM-Manipulation im Browser. npm create Befehl ermöglicht es Ihnen, einfach eine Umgebung zu bauen und beginnen RxJS sofort zu lernen."
---

# Wie man eine Laufzeitumgebung für praktisches Lernen erstellt

Diese Seite bietet eine Entwicklungsvorlage [`RxJS-with-TypeScript-Starter-Kit`](https://github.com/shuji-bonji/rxjs-with-typescript-starter-kit).

## Merkmale

- Einfache Konfiguration von Vite + TypeScript + RxJS
- Hot-Reloading-Unterstützung (führen Sie `npm run dev` aus, um zu prüfen, ob es sofort funktioniert)
- Lokale Entwicklungsumgebung für DOM-Manipulationen und Tests
- Unterstützt Test Driven Development (TDD) mit Vitest

## So verwenden Sie

Die folgenden Befehle können verwendet werden, um das System zu klonen und einzurichten.

```bash
git clone https://github.com/shuji-bonji/rxjs-with-typescript-starter-kit.git
cd rxjs-with-typescript-starter-kit
npm install
npm run dev
```

Der Browser wird automatisch gestartet und der Code in `src/main.ts` wird ausgeführt.

## Beispiele für die Verwendung

Schreiben Sie die bestehende `src/main.ts` wie folgt um.

```ts
// src/main.ts
import { interval } from 'rxjs';
import { take } from 'rxjs';

const output = document.createElement('div');
document.body.appendChild(output);

interval(1000).pipe(take(5)).subscribe((val) => {
  const p = document.createElement('p');
  p.textContent = `Zahl: ${val}`;
  output.appendChild(p);
});
```

### Zugriff auf localhost
Sie werden `http://localhost:5174/` wie folgt sehen, also greifen Sie hier zu, um die Ergebnisse zu überprüfen.
Um die Ergebnisse von `console.log()` zu überprüfen, verwenden Sie die Konsole der Entwicklertools.

```sh
% npm run dev

> rxjs-with-typescript-starter-kit@0.0.0 dev
> vite

Port 5173 ist in Gebrauch, versuchen Sie einen anderen...

  VITE v6.3.1 bereit in 107 ms

  ➜ Lokal: http://localhost:5174/
  ➜ Netzwerk: verwenden Sie --host, um zu exponieren
  ➜ drücke h + enter um Hilfe anzuzeigen
```

## Empfohlene Verwendung

- Experimentieren mit Observable / Subject / Operator
- Erlernen reaktiver UI in Kombination mit DOM
- Übung zur Einführung von Marble-Tests (`vitest` + `TestScheduler`)
- Basisumgebung zum Speichern von Code-Schnipseln

## Link

🔗 Vorlage hier → [RxJS-with-TypeScript-Starter-Kit](https://github.com/shuji-bonji/rxjs-with-typescript-starter-kit)
Siehe `README.md` für weitere Informationen.
