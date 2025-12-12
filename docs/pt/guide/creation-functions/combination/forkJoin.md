---
description: A Creation Function forkJoin emite o último valor de cada Observable como um array ou objeto após todos os múltiplos Observables terem completado. Ideal quando múltiplas requisições de API são executadas em paralelo e todos os resultados estão disponíveis antes do processamento.
---

# forkJoin - emitir todos os últimos valores juntos

`forkJoin` é uma Creation Function que emite o último valor de cada Observable como um array ou objeto após **todos** os Observables terem completado.
Isso é muito útil quando você deseja usar todos os Observables de uma vez.


## Sintaxe básica e uso

```ts
import { forkJoin, of } from 'rxjs';
import { delay } from 'rxjs';

const user$ = of('Usuário A').pipe(delay(1000));
const posts$ = of('Lista de posts').pipe(delay(1500));

forkJoin([user$, posts$]).subscribe(([user, posts]) => {
  console.log(user, posts);
});

// Saída:
// Usuário A Lista de posts
```

- Aguarda até que todos os Observables estejam `complete`.
- Apenas o **último valor emitido** de cada Observable é compilado e emitido.

[🌐 Documentação Oficial RxJS - `forkJoin`](https://rxjs.dev/api/index/function/forkJoin)


## Padrões típicos de utilização

- **Executar múltiplas requisições de API em paralelo e resumir todos os resultados**
- **Obter múltiplos conjuntos de dados necessários para carregamento inicial de uma vez**
- **Obter todos os dados relevantes de uma vez e renderizar a tela de uma só vez**


## Exemplos práticos de código (com UI)

Simula múltiplas requisições de API e exibe os resultados juntos quando todos estiverem disponíveis.

```ts
import { forkJoin, of } from 'rxjs';
import { delay } from 'rxjs';

// Criar área de saída
const output = document.createElement('div');
output.innerHTML = '<h3>Exemplo prático de forkJoin:</h3>';
document.body.appendChild(output);

// Fluxos de dados fictícios
const user$ = of({ id: 1, name: 'João Silva' }).pipe(delay(2000));
const posts$ = of([{ id: 1, title: 'Post 1' }, { id: 2, title: 'Post 2' }]).pipe(delay(1500));
const weather$ = of({ temp: 22, condition: 'Ensolarado' }).pipe(delay(1000));

// Mensagem de carregamento
const loading = document.createElement('div');
loading.textContent = 'Carregando dados...';
loading.style.color = 'blue';
output.appendChild(loading);

// Emitir tudo de uma vez após todas as requisições completarem
forkJoin({
  user: user$,
  posts: posts$,
  weather: weather$
}).subscribe(result => {
  output.removeChild(loading);

  const pre = document.createElement('pre');
  pre.textContent = JSON.stringify(result, null, 2);
  pre.style.background = '#f5f5f5';
  pre.style.padding = '10px';
  pre.style.borderRadius = '5px';
  output.appendChild(pre);

  const summary = document.createElement('div');
  summary.textContent = `Usuário: ${result.user.name}, Clima: ${result.weather.condition}, Posts: ${result.posts.length}`;
  output.appendChild(summary);
});
```

- Exibir carregamento primeiro,
- Quando todos os dados estiverem disponíveis, os resultados serão renderizados juntos.
