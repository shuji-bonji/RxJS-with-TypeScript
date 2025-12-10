---
description: Esta guía explica cómo configurar una plantilla de desarrollo de aprendizaje construida con Vite, TypeScript y RxJS. Incluye recarga en caliente, ideal para experimentar con código y manipular el DOM en el navegador, así como desarrollo dirigido por pruebas usando Vitest.
---

# Configuración del Entorno de Aprendizaje Práctico

Esta página explica cómo usar la plantilla de desarrollo [`RxJS-with-TypeScript-Starter-Kit`](https://github.com/shuji-bonji/rxjs-with-typescript-starter-kit), que te permite probar instantáneamente código RxJS localmente en lugar de en un navegador.

## Características

- Configuración simple: Vite + TypeScript + RxJS
- Soporte de recarga en caliente (ejecuta `npm run dev` para probar inmediatamente)
- Entorno de desarrollo local que soporta manipulación del DOM y pruebas
- Desarrollo Dirigido por Pruebas (TDD) habilitado con Vitest

## Uso

Clona y configura usando los siguientes comandos:

```bash
git clone https://github.com/shuji-bonji/rxjs-with-typescript-starter-kit.git
cd rxjs-with-typescript-starter-kit
npm install
npm run dev
```

Tu navegador se iniciará automáticamente y ejecutará el código escrito en `src/main.ts`.

## Ejemplo de Uso

Reescribe el archivo `src/main.ts` existente de la siguiente manera.

```ts
// src/main.ts
import { interval } from 'rxjs';
import { take } from 'rxjs';

const output = document.createElement('div');
document.body.appendChild(output);

interval(1000).pipe(take(5)).subscribe((val) => {
  const p = document.createElement('p');
  p.textContent = `Conteo: ${val}`;
  output.appendChild(p);
});
```

### Acceder a localhost
La URL se mostrará como `http://localhost:5174/`. Accede a esta URL para ver los resultados.
Para verificar los resultados de `console.log()`, usa la consola en tus herramientas de desarrollador.

```sh
% npm run dev

> rxjs-with-typescript-starter-kit@0.0.0 dev
> vite

Port 5173 is in use, trying another one...

  VITE v6.3.1  ready in 107 ms

  ➜  Local:   http://localhost:5174/
  ➜  Network: use --host to expose
  ➜  press h + enter to show help
```

## Usos Recomendados

- Experimentar con Observables / Subjects / Operadores
- Aprender UI reactiva combinada con DOM
- Practicar implementación de marble tests (`vitest` + `TestScheduler`)
- Entorno base para almacenar tus propios fragmentos de código

## Enlaces

🔗 Plantilla aquí → [RxJS-with-TypeScript-Starter-Kit](https://github.com/shuji-bonji/rxjs-with-typescript-starter-kit)
Ver `README.md` para más detalles.
