---
description: switchMap é um operador de conversão que cancela o Observable anterior e muda para o mais recente. É ideal para casos de uso como pesquisa ao vivo, mudança de navegação, salvamento automático, etc. Junto com a inferência de tipo TypeScript, permite processamento assíncrono seguro. Também fornece explicações detalhadas sobre como usá-lo em conjunto com mergeMap e concatMap.
titleTemplate: ':title'
---

# switchMap - Mudar para o ultimo

O operador `switchMap` cria um novo Observable para cada valor no stream de entrada, **cancelando o Observable anterior e mudando apenas para o mais recente**.
Isso é ideal para casos onde apenas a entrada mais recente deve ser válida, como em um formulário de pesquisa.

## 🔰 Sintaxe Básica e Uso

```ts
import { of } from 'rxjs';
import { delay, switchMap } from 'rxjs';

of('A', 'B', 'C').pipe(
  switchMap(value =>
    of(`${value} concluído`).pipe(delay(1000))
  )
).subscribe(console.log);

// Exemplo de saída:
// C concluído
```

- Cria um novo Observable para cada valor.
- No entanto, **no momento em que um novo valor chega, o Observable anterior é cancelado**.
- Apenas `C` será gerado no final.

[🌐 Documentação Oficial RxJS - `switchMap`](https://rxjs.dev/api/operators/switchMap)

## 💡 Padrões Típicos de Uso

- Autocompletar de formulários de entrada
- Função de pesquisa ao vivo (apenas a entrada mais recente é válida)
- Carregamento de recursos ao mudar navegação ou roteamento
- Mudar ações do usuário para a mais recente

## 🧠 Exemplo de Código Prático (com UI)

Quando um usuário digita texto na caixa de pesquisa, uma requisição de API é enviada imediatamente, exibindo **resultados apenas da última entrada**.

```ts
import { fromEvent } from 'rxjs';
import { debounceTime, map, switchMap } from 'rxjs';
import { ajax } from 'rxjs/ajax';
import { of } from 'rxjs';

// Criar campo de entrada
const searchInput = document.createElement('input');
searchInput.placeholder = 'Pesquisar por nome de usuário';
document.body.appendChild(searchInput);

// Área de saída
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Processamento de evento de entrada
fromEvent(searchInput, 'input').pipe(
  debounceTime(300),
  map(event => (event.target as HTMLInputElement).value.trim()),
  switchMap(term => {
    if (term === '') {
      return of([]);
    }
    return ajax.getJSON(`https://jsonplaceholder.typicode.com/users?username_like=${term}`);
  })
).subscribe(users => {
  output.innerHTML = '';

  (users as any[]).forEach(user => {
    const div = document.createElement('div');
    div.textContent = user.username;
    output.appendChild(div);
  });
});
```

- Cada vez que a entrada muda, a requisição anterior é cancelada.
- Apenas usuários correspondentes ao termo de pesquisa mais recente serão exibidos.
