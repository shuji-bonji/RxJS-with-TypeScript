---
description: O operador exhaustMap é um operador de conversão que ignora novas entradas até que o Observable atualmente em processamento seja concluído. É eficaz em situações onde você deseja limitar a simultaneidade, como prevenir múltiplos cliques no botão de envio de formulário ou submissões duplicadas de requisições de API.
---

# exhaustMap - Ignora Nova Entrada Durante a Execução

O operador `exhaustMap` **ignora nova entrada** até que o Observable atualmente em processamento seja concluído.
Isso é ideal para prevenir cliques duplicados ou múltiplas submissões de requisições.

## 🔰 Sintaxe Básica e Uso

```ts
import { fromEvent, of } from 'rxjs';
import { exhaustMap, delay } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$
  .pipe(exhaustMap(() => of('Requisição concluída').pipe(delay(1000))))
  .subscribe(console.log);

// Exemplo de saída:
// (Apenas o primeiro clique gera "Requisição concluída" após 1 segundo)

```

- A entrada subsequente é ignorada até que a requisição em execução seja concluída.

[🌐 Documentação Oficial RxJS - `exhaustMap`](https://rxjs.dev/api/operators/exhaustMap)

## 💡 Padrões Típicos de Uso

- Prevenção de múltiplos cliques em botões de envio de formulário
- Prevenção de requisições duplas (especialmente para processos de login e pagamento)
- Controle de exibição única de modal ou diálogo

## 🧠 Exemplo de Código Prático (com UI)

Clicar no botão Enviar inicia o processo de envio.
**Não importa quantas vezes você clique durante a transmissão, será ignorado** e a próxima transmissão não será aceita até que o primeiro processo seja concluído.

```ts
import { fromEvent } from 'rxjs';
import { exhaustMap, delay } from 'rxjs';
import { ajax } from 'rxjs/ajax';

// Criar botão
const submitButton = document.createElement('button');
submitButton.textContent = 'Enviar';
document.body.appendChild(submitButton);

// Criar área de saída
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Processamento de envio
fromEvent(submitButton, 'click')
  .pipe(
    exhaustMap(() => {
      output.textContent = 'Enviando...';
      return ajax
        .post('https://jsonplaceholder.typicode.com/posts', {
          title: 'foo',
          body: 'bar',
          userId: 1,
        })
        .pipe(delay(2000)); // Simula atraso de envio de 2 segundos
    })
  )
  .subscribe({
    next: (response) => {
      output.textContent = 'Envio bem-sucedido!';
      console.log('Envio bem-sucedido:', response);
    },
    error: (error) => {
      output.textContent = 'Erro de envio';
      console.error('Erro de envio:', error);
    },
  });

```

- Quaisquer outros cliques enquanto o botão estiver sendo clicado serão ignorados.
- Após 2 segundos, você verá "Envio bem-sucedido!" ou "Erro de envio" será exibido.
