---
Description: Este guia explica como configurar um template de desenvolvimento para aprendizado construído com Vite, TypeScript e RxJS. Possui hot reloading, tornando-o ideal para experimentar com código e manipular o DOM no navegador, bem como desenvolvimento orientado a testes usando Vitest.
---

# Configurando o Ambiente de Aprendizado Prático

Esta página explica como usar o template de desenvolvimento [`RxJS-with-TypeScript-Starter-Kit`](https://github.com/shuji-bonji/rxjs-with-typescript-starter-kit), que permite testar código RxJS instantaneamente localmente ao invés de em um navegador.

## Características

- Configuração simples: Vite + TypeScript + RxJS
- Suporte a hot reload (execute `npm run dev` para testar imediatamente)
- Ambiente de desenvolvimento local com suporte a manipulação de DOM e testes
- Desenvolvimento Orientado a Testes (TDD) habilitado com Vitest

## Uso

Clone e configure usando os seguintes comandos:

```bash
git clone https://github.com/shuji-bonji/rxjs-with-typescript-starter-kit.git
cd rxjs-with-typescript-starter-kit
npm install
npm run dev
```

Seu navegador será iniciado automaticamente e executará o código escrito em `src/main.ts`.

## Exemplo de Uso

Reescreva o `src/main.ts` existente da seguinte forma.

```ts
// src/main.ts
import { interval } from 'rxjs';
import { take } from 'rxjs';

const output = document.createElement('div');
document.body.appendChild(output);

interval(1000).pipe(take(5)).subscribe((val) => {
  const p = document.createElement('p');
  p.textContent = `Count: ${val}`;
  output.appendChild(p);
});
```

### Acesse localhost
A URL será exibida como `http://localhost:5174/`. Acesse esta URL para ver os resultados.
Para verificar os resultados de `console.log()`, use o console nas ferramentas de desenvolvedor.

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

- Experimentar com Observables / Subjects / Operadores
- Aprender UI reativa combinada com DOM
- Praticar implementação de marble tests (`vitest` + `TestScheduler`)
- Ambiente base para armazenar seus próprios snippets de código

## Links

🔗 Template aqui → [RxJS-with-TypeScript-Starter-Kit](https://github.com/shuji-bonji/rxjs-with-typescript-starter-kit)
Veja `README.md` para detalhes.
